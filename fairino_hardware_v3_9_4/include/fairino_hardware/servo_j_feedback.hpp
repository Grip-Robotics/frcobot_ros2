#ifndef FAIRINO_HARDWARE__SERVO_J_FEEDBACK_HPP_
#define FAIRINO_HARDWARE__SERVO_J_FEEDBACK_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace fairino_hardware
{

struct ServoJFeedbackStats
{
  uint64_t published{0};
  uint64_t coalesced{0};
  uint64_t contended{0};
  uint64_t exceptions{0};
  int64_t publication_begin_ns{0};
  int64_t publication_end_ns{0};
  int64_t maximum_publication_ns{0};
  bool in_flight_at_close{false};
};

// One worker for the node lifetime; a blocked publisher never holds a sender lock.
class ServoJFeedbackPublisher
{
public:
  using Callback = std::function<void (uint64_t, uint64_t, int64_t)>;

  class Session
  {
  public:
    explicit Session(Callback callback) : callback_(std::move(callback)) {}

    // Single sender. One lock-free attempt; pending progress may be coalesced.
    void offer(uint64_t samples, uint64_t command, int64_t start_ns) noexcept
    {
      if (state_.load() & closed) {return;}
      if (slot_busy_.test_and_set(std::memory_order_acquire)) {
        contended_.fetch_add(1);
        return;
      }
      if (pending_) {coalesced_.fetch_add(1);}
      latest_ = {samples, command, start_ns};
      pending_ = true;
      slot_busy_.clear(std::memory_order_release);
    }

    // Does not wait for publication. The action result owns the terminal counts.
    ServoJFeedbackStats close() noexcept
    {
      const auto previous = state_.fetch_or(closed);
      return {published_.load(), coalesced_.load(), contended_.load(), exceptions_.load(),
        publication_begin_ns_.load(), publication_end_ns_.load(), maximum_publication_ns_.load(),
        (previous & publishing) != 0};
    }

  private:
    friend class ServoJFeedbackPublisher;
    struct Progress
    {
      uint64_t samples{0};
      uint64_t command{0};
      int64_t start_ns{0};
    };

    static int64_t now_ns()
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    void publish_latest()
    {
      if ((state_.load() & closed) || slot_busy_.test_and_set(std::memory_order_acquire)) {return;}
      const auto progress = latest_;
      const bool pending = pending_;
      pending_ = false;
      slot_busy_.clear(std::memory_order_release);
      unsigned expected = 0;
      if (!pending || !state_.compare_exchange_strong(expected, publishing)) {return;}
      const auto begin = now_ns();
      publication_begin_ns_.store(begin);
      // A call already entering middleware can finish after close(), on its original goal.
      try {
        callback_(progress.samples, progress.command, progress.start_ns);
        published_.fetch_add(1);
      } catch (...) {
        exceptions_.fetch_add(1);
      }
      const auto end = now_ns();
      publication_end_ns_.store(end);
      maximum_publication_ns_.store(
        std::max(maximum_publication_ns_.load(), end - begin));
      state_.fetch_and(~publishing);
    }

    Callback callback_;
    std::atomic_flag slot_busy_ = ATOMIC_FLAG_INIT;
    Progress latest_{};
    bool pending_{false};
    static constexpr unsigned closed = 1, publishing = 2;
    std::atomic<unsigned> state_{0};
    std::atomic<uint64_t> published_{0}, coalesced_{0}, contended_{0}, exceptions_{0};
    std::atomic<int64_t> publication_begin_ns_{0}, publication_end_ns_{0};
    std::atomic<int64_t> maximum_publication_ns_{0};
  };

  ServoJFeedbackPublisher() : worker_([this]() {run();}) {}
  ~ServoJFeedbackPublisher() {shutdown();}

  // Call before raising the sender's scheduling priority, outside the timed loop.
  std::shared_ptr<Session> start(Callback callback)
  {
    auto session = std::make_shared<Session>(std::move(callback));
    std::lock_guard<std::mutex> lock(mutex_);
    if (session_) {session_->close();}
    session_ = session;
    return session;
  }

  void shutdown()
  {
    stopping_.store(true);
    if (worker_.joinable()) {worker_.join();}
  }

private:
  void run()
  {
    while (!stopping_.load()) {
      std::shared_ptr<Session> session;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        session = session_;
      }
      if (session) {session->publish_latest();}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  static_assert(std::atomic<uint64_t>::is_always_lock_free, "feedback counters must be lock-free");
  static_assert(std::atomic<unsigned>::is_always_lock_free, "feedback state must be lock-free");
  std::mutex mutex_;
  std::shared_ptr<Session> session_;
  std::atomic_bool stopping_{false};
  std::thread worker_;
};

}  // namespace fairino_hardware

#endif  // FAIRINO_HARDWARE__SERVO_J_FEEDBACK_HPP_
