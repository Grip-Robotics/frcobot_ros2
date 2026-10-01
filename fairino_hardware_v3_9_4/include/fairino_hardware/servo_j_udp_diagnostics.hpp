#ifndef FAIRINO_HARDWARE__SERVO_J_UDP_DIAGNOSTICS_HPP_
#define FAIRINO_HARDWARE__SERVO_J_UDP_DIAGNOSTICS_HPP_

#include "fairino_hardware/servo_j_diagnostics.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <ostream>
#include <string>

namespace fairino_hardware
{

// One producer and one consumer. A full queue loses diagnostics, never delays
// either SDK thread. The consumer is serialized outside the servo loop.
template<class T, size_t Capacity>
class ServoDiagnosticQueue
{
public:
  static_assert(Capacity > 1, "diagnostic queue needs at least two slots");
  static_assert(std::atomic<size_t>::is_always_lock_free &&
    std::atomic<uint64_t>::is_always_lock_free, "diagnostic atomics must be lock-free");
  bool push(const T & value) noexcept
  {
    const size_t write = write_.load(std::memory_order_relaxed);
    const size_t next = (write + 1) % Capacity;
    if (next == read_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    values_[write] = value;
    write_.store(next, std::memory_order_release);
    return true;
  }

  bool pop(T & value) noexcept
  {
    const size_t read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) {return false;}
    value = values_[read];
    read_.store((read + 1) % Capacity, std::memory_order_release);
    return true;
  }

  uint64_t dropped() const noexcept {return dropped_.load(std::memory_order_relaxed);}

private:
  std::array<T, Capacity> values_{};
  std::atomic<size_t> write_{0};
  std::atomic<size_t> read_{0};
  std::atomic<uint64_t> dropped_{0};
};

struct ServoUdpCall
{
  int64_t mono_ns{0};
  int frame_command{0};
  int command_id{-1};
  int sdk_error{0};
  bool failure{false};
};

struct ServoUdpReply
{
  int64_t mono_ns{0};
  int source{1};
  int counter{0};
  int frame_command{0};
  int content_length{0};
  size_t actual_length{0};
  size_t retained_length{0};
  std::array<char, 64> content{};
};

// This state is accessed only by the ordinary trace writer, never by either
// SDK thread. Frame counters are eight bits; a unique pending candidate is an
// inference, not controller proof of acceptance or physical completion.
class ServoUdpTracker
{
public:
  struct Snapshot
  {
    uint64_t session{0};
    int64_t start_call_ns{0};
    int64_t start_reply_ns{0};
    int start_counter{-1};
    int64_t end_call_ns{0};
    int64_t end_reply_ns{0};
    uint64_t calls{0};
    uint64_t matched_replies{0};
    uint64_t last_replied_ordinal{0};
    int last_replied_command_id{-1};
    int64_t last_reply_rtt_ns{0};
    int64_t maximum_reply_rtt_ns{0};
    uint64_t unmatched{0};
    uint64_t ambiguous{0};
    uint64_t stale{0};
    uint64_t malformed{0};
    uint64_t out_of_order{0};
    uint64_t previous_unreplied{0};
    bool previous_start_unreplied{false};
    bool degraded{false};
  };

  void call(const ServoUdpCall & event)
  {
    if (event.failure) {
      if (event.mono_ns >= state_.start_call_ns) {state_.degraded = true;}
      return;
    }
    if (event.frame_command == 689) {
      const uint64_t session = state_.session + 1;
      const uint64_t previous_unreplied = state_.calls - state_.matched_replies;
      const bool previous_start_unreplied = state_.start_call_ns && !state_.start_reply_ns;
      state_ = Snapshot{};
      state_.session = session;
      state_.start_call_ns = event.mono_ns;
      state_.previous_unreplied = previous_unreplied;
      state_.previous_start_unreplied = previous_start_unreplied;
      history_count_ = 0;
      return;
    }
    if (!state_.session) {return;}
    if (event.frame_command == 690) {state_.end_call_ns = event.mono_ns; return;}
    if (event.frame_command != 376) {state_.degraded = true; return;}
    const uint64_t ordinal = ++state_.calls;
    auto & pending = pending_[ordinal % pending_.size()];
    if (ordinal > pending_.size() && pending.session == state_.session && !pending.replied) {
      state_.degraded = true;
    }
    pending = Pending{state_.session, ordinal, event.mono_ns, event.command_id, false};
  }

  void degrade() {state_.degraded = true;}
  const Snapshot & snapshot() const {return state_;}

  void reply(const ServoUdpReply & event)
  {
    Record record{};
    record.reply = event;
    record.session = state_.session;
    if (event.source == 1 && event.mono_ns >= state_.start_call_ns &&
      event.frame_command != 376 && event.frame_command != 689 && event.frame_command != 690)
    {
      // Another UDP SDK operation can consume an envelope counter. Do not
      // silently keep predicting ServoJ counters after observing one.
      state_.degraded = true;
    }
    if (event.counter < 0 || event.counter > 255 || event.content_length < 0 ||
      static_cast<size_t>(event.content_length) != event.actual_length)
    {
      ++state_.malformed;
      record.match = "malformed";
    } else if (!state_.session || event.mono_ns < state_.start_call_ns) {
      ++state_.stale;
      record.match = "before_session";
    } else if (event.source != 1) {
      ++state_.unmatched;
      record.match = "other_transport";
    } else if (event.frame_command == 689 && !state_.start_reply_ns) {
      if (state_.previous_start_unreplied || state_.degraded) {
        ++state_.ambiguous;
        record.match = "ambiguous_start";
      } else {
        state_.start_reply_ns = event.mono_ns;
        state_.start_counter = event.counter;
        record.call_ns = state_.start_call_ns;
        record.match = "start_reply";
      }
    } else if (event.frame_command == 690 && state_.end_call_ns &&
      event.mono_ns >= state_.end_call_ns && state_.start_counter >= 0 &&
      event.counter == static_cast<int>((state_.start_counter + state_.calls + 1) % 256) &&
      !state_.degraded && !state_.end_reply_ns)
    {
      state_.end_reply_ns = event.mono_ns;
      record.call_ns = state_.end_call_ns;
      record.match = "end_reply";
    } else if (event.frame_command == 376 && state_.start_counter >= 0 && !state_.degraded) {
      // At most 16 candidates in the bounded 4096-call window. An unreturned
      // old call plus a new call with the same counter must remain ambiguous.
      const uint64_t residue = (event.counter - state_.start_counter + 256) % 256;
      uint64_t ordinal = residue ? residue : 256;
      const uint64_t first = state_.calls > pending_.size() ?
        state_.calls - pending_.size() + 1 : 1;
      if (ordinal < first) {ordinal += ((first - ordinal + 255) / 256) * 256;}
      Pending * candidate = nullptr;
      size_t candidates = 0;
      for (; ordinal <= state_.calls; ordinal += 256) {
        auto & pending = pending_[ordinal % pending_.size()];
        if (pending.session == state_.session && pending.ordinal == ordinal &&
          !pending.replied && pending.mono_ns <= event.mono_ns)
        {
          candidate = &pending;
          ++candidates;
        }
      }
      if (candidates == 1) {
        candidate->replied = true;
        record.ordinal = candidate->ordinal;
        record.command_id = candidate->command_id;
        record.call_ns = candidate->mono_ns;
        record.match = "unique_pending_counter";
        ++state_.matched_replies;
        if (record.ordinal < state_.last_replied_ordinal) {++state_.out_of_order;} else {
          state_.last_replied_ordinal = record.ordinal;
          state_.last_replied_command_id = record.command_id;
        }
        state_.last_reply_rtt_ns = event.mono_ns - record.call_ns;
        state_.maximum_reply_rtt_ns = std::max(
          state_.maximum_reply_rtt_ns, state_.last_reply_rtt_ns);
      } else if (candidates > 1) {
        ++state_.ambiguous;
        record.match = "ambiguous_counter_wrap";
      } else {
        ++state_.unmatched;
        record.match = "no_pending_counter";
      }
    } else {
      ++state_.unmatched;
      record.match = state_.degraded ? "mapping_degraded" : "unmatched";
    }
    history_[history_count_++ % history_.size()] = record;
  }

  void write_summary(std::ostream & out) const
  {
    out << "{\"correlation_inferred\":true,\"physical_completion_known\":false," <<
      "\"session\":" << state_.session << ",\"start_call_mono_ns\":" << state_.start_call_ns <<
      ",\"start_reply_mono_ns\":" << state_.start_reply_ns <<
      ",\"start_frame_counter\":" << state_.start_counter << ",\"start_reply_latency_ns\":";
    if (state_.start_reply_ns) {out << state_.start_reply_ns - state_.start_call_ns;} else {
      out << "null";
    }
    out << ",\"end_call_mono_ns\":" << state_.end_call_ns <<
      ",\"end_reply_mono_ns\":" << state_.end_reply_ns <<
      ",\"servo_calls\":" << state_.calls <<
      ",\"matched_servo_replies\":" << state_.matched_replies <<
      ",\"calls_without_matched_reply\":" << state_.calls - state_.matched_replies <<
      ",\"last_replied_ordinal\":" << state_.last_replied_ordinal <<
      ",\"last_replied_command_id\":" << state_.last_replied_command_id <<
      ",\"last_reply_rtt_ns\":" << state_.last_reply_rtt_ns <<
      ",\"maximum_reply_rtt_ns\":" << state_.maximum_reply_rtt_ns <<
      ",\"unmatched_replies\":" << state_.unmatched <<
      ",\"ambiguous_replies\":" << state_.ambiguous <<
      ",\"before_session_replies\":" << state_.stale <<
      ",\"malformed_replies\":" << state_.malformed <<
      ",\"out_of_order_replies\":" << state_.out_of_order <<
      ",\"previous_session_unreplied_calls\":" << state_.previous_unreplied <<
      ",\"previous_start_unreplied\":" << (state_.previous_start_unreplied ? "true" : "false") <<
      ",\"mapping_degraded\":" << (state_.degraded ? "true" : "false") <<
      ",\"reply_history_overwritten\":" <<
      (history_count_ > history_.size() ? history_count_ - history_.size() : 0) << '}';
  }

  void write_history(std::ostream & out) const
  {
    const uint64_t first = history_count_ > history_.size() ?
      history_count_ - history_.size() : 0;
    for (uint64_t index = first; index < history_count_; ++index) {
      const auto & record = history_[index % history_.size()];
      const auto & reply = record.reply;
      out << "{\"event\":\"servo_udp_reply\",\"session\":" << record.session <<
        ",\"receive_mono_ns\":" << reply.mono_ns << ",\"source\":" << reply.source <<
        ",\"frame_counter\":" << reply.counter << ",\"frame_command\":" << reply.frame_command <<
        ",\"content_length\":" << reply.content_length <<
        ",\"content_truncated\":" << (reply.actual_length >
      reply.content.size() ? "true" : "false") <<
        ",\"content\":";
      json_string(out, std::string(reply.content.data(), reply.retained_length));
      out << ",\"match\":";
      json_string(out, record.match);
      out << ",\"inferred_servo_ordinal\":" << record.ordinal <<
        ",\"inferred_command_id\":" << record.command_id <<
        ",\"call_mono_ns\":" << record.call_ns << ",\"reply_latency_ns\":";
      if (record.call_ns) {out << reply.mono_ns - record.call_ns;} else {out << "null";}
      out << "}\n";
    }
  }

private:
  struct Pending
  {
    uint64_t session{0};
    uint64_t ordinal{0};
    int64_t mono_ns{0};
    int command_id{-1};
    bool replied{false};
  };
  struct Record
  {
    ServoUdpReply reply{};
    uint64_t session{0};
    uint64_t ordinal{0};
    int command_id{-1};
    int64_t call_ns{0};
    const char * match{"unmatched"};
  };
  Snapshot state_{};
  std::array<Pending, 4096> pending_{};
  std::array<Record, 8192> history_{};
  uint64_t history_count_{0};
};

class ServoUdpDiagnostics
{
public:
  // A driver process has exactly one SDK UDP receiver. This mailbox is never
  // re-enabled after shutdown: the SDK owns detached receiver threads and its
  // callback has no context pointer or supported nullptr unregister operation.
  bool enable_once() noexcept
  {
    bool expected = false;
    if (!used_.compare_exchange_strong(expected, true)) {return false;}
    enabled_.store(true, std::memory_order_release);
    return true;
  }
  void disable() noexcept {enabled_.store(false, std::memory_order_release);}
  bool enabled() const noexcept {return enabled_.load(std::memory_order_acquire);}

  void call(int frame_command, int command_id, int64_t mono_ns) noexcept
  {
    if (enabled()) {calls_.push(ServoUdpCall{mono_ns, frame_command, command_id, 0, false});}
  }
  void failure(int frame_command, int error, int64_t mono_ns) noexcept
  {
    if (enabled()) {calls_.push(ServoUdpCall{mono_ns, frame_command, -1, error, true});}
  }
  void receive(
    int source, int counter, int command, int length,
    const std::string & content, int64_t mono_ns) noexcept
  {
    if (!enabled()) {return;}
    ServoUdpReply event{};
    event.mono_ns = mono_ns;
    event.source = source;
    event.counter = counter;
    event.frame_command = command;
    event.content_length = length;
    event.actual_length = content.size();
    event.retained_length = std::min(content.size(), event.content.size());
    std::memcpy(event.content.data(), content.data(), event.retained_length);
    replies_.push(event);
  }

  // Called from the 20 Hz state writer and after a stream, never from the
  // sender or SDK receive callback. Only these readers acquire the mutex.
  void write_summary(std::ostream & out)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    drain();
    out << "{\"enabled\":" << (enabled() ? "true" : "false") <<
      ",\"call_queue_dropped\":" << calls_.dropped() <<
      ",\"reply_queue_dropped\":" << replies_.dropped() << ",\"tracking\":";
    tracker_.write_summary(out);
    out << '}';
  }
  void write_history(std::ostream & out)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    drain();
    tracker_.write_history(out);
  }

  static int64_t now_ns() noexcept
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

private:
  void drain()
  {
    // Snapshot RX first. Every request capable of producing one of these
    // replies was published before its SDK call began. Snapshot TX next, then
    // merge by monotonic time so adjacent short sessions remain separate.
    size_t received = 0;
    while (received < scratch_.size() && replies_.pop(scratch_[received])) {++received;}
    size_t called = 0;
    while (called < call_scratch_.size() && calls_.pop(call_scratch_[called])) {++called;}
    const bool lost = calls_.dropped() != last_call_drops_ ||
      replies_.dropped() != last_reply_drops_;
    if (lost) {
      tracker_.degrade();
      last_call_drops_ = calls_.dropped();
      last_reply_drops_ = replies_.dropped();
    }
    size_t call_index = 0;
    size_t reply_index = 0;
    while (call_index < called || reply_index < received) {
      if (call_index < called && (reply_index == received ||
        call_scratch_[call_index].mono_ns <= scratch_[reply_index].mono_ns))
      {
        tracker_.call(call_scratch_[call_index++]);
        if (lost) {tracker_.degrade();}
      } else {tracker_.reply(scratch_[reply_index++]);}
    }
  }
  std::atomic<bool> used_{false};
  std::atomic<bool> enabled_{false};
  ServoDiagnosticQueue<ServoUdpCall, 2048> calls_;
  ServoDiagnosticQueue<ServoUdpReply, 2048> replies_;
  std::array<ServoUdpReply, 2048> scratch_{};
  std::array<ServoUdpCall, 2048> call_scratch_{};
  std::mutex mutex_;
  ServoUdpTracker tracker_;
  uint64_t last_call_drops_{0};
  uint64_t last_reply_drops_{0};
};

}  // namespace fairino_hardware

#endif  // FAIRINO_HARDWARE__SERVO_J_UDP_DIAGNOSTICS_HPP_
