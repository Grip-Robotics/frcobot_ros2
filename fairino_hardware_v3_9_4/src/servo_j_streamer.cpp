#include "fairino_hardware/servo_j_streamer.hpp"
#include "fairino_hardware/servo_j_diagnostics.hpp"

#include <cerrno>
#include <algorithm>
#include <cmath>
#include <climits>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace
{

double nanoseconds_to_milliseconds(int64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / 1.0e6;
}

}  // namespace

namespace fairino_hardware
{

int64_t SystemMonotonicClock::now_nanoseconds()
{
  timespec now{};
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
    return -1;
  }
  return static_cast<int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
}

int SystemMonotonicClock::sleep_until(int64_t absolute_nanoseconds)
{
  timespec deadline{};
  deadline.tv_sec = absolute_nanoseconds / 1000000000LL;
  deadline.tv_nsec = absolute_nanoseconds % 1000000000LL;
  int result;
  do {
    result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
  } while (result == EINTR);
  return result;
}

ServoJStreamer::ServoJStreamer(
  ServoJRobot & robot, MonotonicClock & clock, ServoJStreamerConfig config)
: robot_(robot), clock_(clock), config_(config),
  trace_(std::min<size_t>(config.trace_capacity, 32768))
{
}

bool ServoJStreamer::validate(const ServoJStreamRequest & request, std::string & reason) const
{
  if (!config_.servo_start.validate(reason)) {return false;}
  if (!std::isfinite(config_.minimum_period_sec) ||
    !std::isfinite(config_.maximum_period_sec) || config_.minimum_period_sec <= 0.0 ||
    config_.maximum_period_sec < config_.minimum_period_sec ||
    !std::isfinite(config_.maximum_joint_step_deg) || config_.maximum_joint_step_deg <= 0.0 ||
    !std::isfinite(config_.deadline_tolerance_sec) || config_.deadline_tolerance_sec < 0.0 ||
    !std::isfinite(config_.maximum_lateness_sec) ||
    !std::isfinite(config_.feedback_period_sec) || config_.feedback_period_sec < 0.0 ||
    !std::isfinite(config_.sdk_cmdt_override_sec) || config_.sdk_cmdt_override_sec < 0.0 ||
    (config_.sdk_cmdt_override_sec > 0.0 &&
    (config_.sdk_cmdt_override_sec < config_.minimum_period_sec ||
    config_.sdk_cmdt_override_sec > config_.maximum_period_sec)))
  {
    reason = "invalid ServoJ driver configuration";
    return false;
  }
  if (request.joints_deg.empty() || request.joints_deg.size() % 6 != 0) {
    reason = "joints_deg must contain one or more complete six-joint samples";
    return false;
  }
  if (!std::isfinite(request.period_sec) ||
    request.period_sec < config_.minimum_period_sec ||
    request.period_sec > config_.maximum_period_sec)
  {
    std::ostringstream message;
    message << "period_sec must be finite and in [" << config_.minimum_period_sec << ", " <<
      config_.maximum_period_sec << "]";
    reason = message.str();
    return false;
  }
  if (request.communication_type != 1) {
    reason = "communication_type must be 1 (FAIRINO UDP port 20007)";
    return false;
  }
  for (size_t index = 0; index < request.joints_deg.size(); ++index) {
    if (!std::isfinite(request.joints_deg[index])) {
      reason = "every joint target must be finite";
      return false;
    }
    if (index >= 6 &&
      std::abs(request.joints_deg[index] - request.joints_deg[index - 6]) >
      config_.maximum_joint_step_deg)
    {
      std::ostringstream message;
      message << "adjacent samples exceed the " << config_.maximum_joint_step_deg <<
        " degree ServoJ command limit at sample " << (index / 6) << ", joint " <<
        (index % 6);
      reason = message.str();
      return false;
    }
  }
  if (config_.queue.enabled) {
    const auto & queue = config_.queue;
    if (!queue.read || queue.prefix_samples < 2 ||
      queue.prefix_samples >= request.joints_deg.size() / 6 ||
      queue.target_queue < 2 || queue.target_queue > 29 || queue.maximum_queue > 32 ||
      queue.maximum_queue < queue.target_queue + 3 ||
      std::abs(request.period_sec - 0.006006) > 1e-12 ||
      std::abs(config_.sdk_cmdt_override_sec - 0.006) > 1e-12)
    {
      reason = "invalid experimental queue configuration (requires 6.006/6 ms)";
      return false;
    }
    for (size_t index = 6; index < queue.prefix_samples * 6; ++index) {
      if (request.joints_deg[index] != request.joints_deg[index % 6]) {
        reason = "queue prefill prefix must contain identical stationary targets";
        return false;
      }
    }
  }
  return true;
}

bool ServoJStreamer::reserve(const ServoJStreamRequest & request, std::string & reason)
{
  if (!validate(request, reason)) {
    return false;
  }
  bool expected = false;
  if (!active_.compare_exchange_strong(expected, true)) {
    reason = "another ServoJ stream is already active";
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    buffer_ = std::make_shared<const std::vector<double>>(request.joints_deg);
    goal_id_ = request.goal_id;
    sealed_ = false;
    revision_ = next_sample_ = buffer_offset_ = 0;
    period_sec_ = request.period_sec;
    sdk_cmdt_sec_ = config_.sdk_cmdt_override_sec > 0.0 ?
      config_.sdk_cmdt_override_sec : request.period_sec;
  }
  cancel_requested_.store(false);
  {
    std::lock_guard<std::mutex> stop_lock(stop_mutex_);
    stop_called_ = false;
    stop_error_ = 0;
  }
  return true;
}

bool ServoJStreamer::replace_tail(
  const std::string & goal_id,
  uint64_t expected_revision, uint64_t start_sample, const std::vector<double> & tail,
  uint64_t & revision, std::string & reason)
{
  if (config_.queue.enabled) {
    reason = "tail replacement is disabled for experimental queue regulation";
    return false;
  }
  std::shared_ptr<const std::vector<double>> original;
  double period;
  uint64_t offset, retire;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (!active_ || sealed_ || cancel_requested_ || goal_id.empty() || goal_id != goal_id_ ||
      revision_ != expected_revision || !buffer_ || start_sample < buffer_offset_ ||
      start_sample >= buffer_offset_ + buffer_->size() / 6)
    {
      reason = "inactive stream, stale revision, or invalid splice";
      return false;
    }
    original = buffer_;
    period = period_sec_;
    offset = buffer_offset_;
    retire = std::max(offset, next_sample_ > 0 ? next_sample_ - 1 : 0);
  }
  if (tail.empty() || tail.size() % 6 != 0) {
    reason = "replacement requires complete joint samples";
    return false;
  }
  auto updated = std::make_shared<std::vector<double>>();
  if (start_sample <= retire) {reason = "splice is already consumed"; return false;}
  updated->reserve((start_sample - retire) * 6 + tail.size());
  updated->insert(updated->end(), original->begin() + (retire - offset) * 6,
    original->begin() + (start_sample - offset) * 6);
  updated->insert(updated->end(), tail.begin(), tail.end());
  ServoJStreamRequest check;
  check.joints_deg = *updated;
  check.period_sec = period;
  if (!validate(check, reason)) {return false;}
  {
    // Allocation and validation happen outside the realtime stream lock.
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    revision = revision_;
    const uint64_t guard = static_cast<uint64_t>(std::ceil(0.1 / period_sec_));
    if (!active_ || sealed_ || cancel_requested_ || goal_id != goal_id_ ||
      revision_ != expected_revision || buffer_ != original ||
      start_sample <= next_sample_ + guard)
    {
      reason = "continuation deadline passed or stream changed; fallback retained";
      return false;
    }
    buffer_ = updated;
    buffer_offset_ = retire;
    revision = ++revision_;
  }
  return true;
}

int ServoJStreamer::stop_once()
{
  // Keep later cleanup callers behind the first StopMotion call so ServoMoveEnd
  // can never overtake a cancellation thread that is waiting for the SDK lock.
  std::lock_guard<std::mutex> stop_lock(stop_mutex_);
  if (stop_called_) {
    return stop_error_;
  }
  stop_called_ = true;
  stop_error_ = robot_.stop_motion();
  return stop_error_;
}

void ServoJStreamer::finish_with_stop_and_end(
  ServoJStreamMetrics & metrics, int communication_type)
{
  const int stop_error = stop_once();
  const int end_error = robot_.servo_move_end(communication_type);
  if (metrics.controller_error == 0) {
    metrics.controller_error = stop_error != 0 ? stop_error : end_error;
  }
}

ServoJStreamMetrics ServoJStreamer::run_reserved(
  const ServoJStreamRequest & request, const FeedbackCallback & feedback,
  const std::function<int64_t()> & ros_now)
{
  ServoJStreamMetrics metrics;
  trace_count_ = 0;
  trace_start_mono_ns_ = trace_start_ros_ns_ = 0;
  metrics.samples_requested = request.joints_deg.size() / 6;
  const int communication_type = request.communication_type;
  // reserve() freezes this value for the complete stream and its revisions.
  // Firmware interpretation/quantization of cmdT is not inferred here.
  double sdk_cmdt_sec;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    sdk_cmdt_sec = sdk_cmdt_sec_;
  }
  metrics.sdk_cmdt_sec = sdk_cmdt_sec;
  const int64_t period_ns = seconds_to_nanoseconds(request.period_sec);
  const int64_t tolerance_ns = seconds_to_nanoseconds(config_.deadline_tolerance_sec);
  const double fatal_lateness = config_.maximum_lateness_sec > 0.0 ?
    config_.maximum_lateness_sec : request.period_sec;
  const int64_t fatal_lateness_ns = seconds_to_nanoseconds(fatal_lateness);
  const int64_t feedback_period_ns = seconds_to_nanoseconds(config_.feedback_period_sec);

  auto release = [this]() {active_.store(false);};
  struct ScopeExit
  {
    std::function<void()> function;
    ~ScopeExit() {function();}
  } scope_exit{release};

  if (cancel_requested_.load()) {
    metrics.cancelled = true;
    metrics.message = "ServoJ stream cancelled";
    return metrics;
  }

  const bool queue_enabled = config_.queue.enabled;
  ServoJQueueSnapshot queue_snapshot{};
  const uint64_t previous_session_sequence = last_queue_snapshot_.sequence;
  auto read_queue = [&]() -> const char * {
      try {
        if (!config_.queue.read(queue_snapshot)) {return "Queue feedback unavailable or faulted";}
      } catch (...) {return "Queue feedback callback failed";}
      const auto now = clock_.now_nanoseconds();
      if (now < 0 || queue_snapshot.sequence == 0 ||
        queue_snapshot.received_mono_ns <= 0 || queue_snapshot.received_mono_ns > now ||
        queue_snapshot.queue_length < 0 || queue_snapshot.queue_length > config_.queue.maximum_queue)
      {return "Invalid queue feedback or queue ceiling exceeded";}
      if (now - queue_snapshot.received_mono_ns > 50000000LL) {
        return "Queue feedback older than 50 ms";
      }
      const auto & previous = last_queue_snapshot_;
      if (queue_snapshot.sequence < previous.sequence ||
        queue_snapshot.received_mono_ns < previous.received_mono_ns ||
        (queue_snapshot.sequence == previous.sequence &&
        (queue_snapshot.received_mono_ns != previous.received_mono_ns ||
        queue_snapshot.queue_length != previous.queue_length)) ||
        (queue_snapshot.sequence > previous.sequence && previous.sequence != 0 &&
        queue_snapshot.received_mono_ns <= previous.received_mono_ns))
      {return "Nonmonotonic or inconsistent queue feedback";}
      last_queue_snapshot_ = queue_snapshot;
      return nullptr;
    };
  if (queue_enabled) {
    const auto failure = read_queue();
    if (failure || queue_snapshot.queue_length != 0 ||
      queue_snapshot.sequence <= previous_session_sequence)
    {
      metrics.message = failure ? failure : "Queue start requires a new empty raw feedback frame";
      return metrics;  // No servo session has started.
    }
    if (cancel_requested_.load()) {
      metrics.cancelled = true;
      metrics.message = "ServoJ stream cancelled";
      return metrics;
    }
  }

  int error = robot_.servo_move_start(communication_type, config_.servo_start);
  if (error != 0) {
    metrics.controller_error = error;
    metrics.cancelled = cancel_requested_.load();
    metrics.message = "ServoMoveStart failed";
    finish_with_stop_and_end(metrics, communication_type);
    return metrics;
  }

  const int64_t start_ns = clock_.now_nanoseconds();
  trace_start_mono_ns_ = start_ns;
  if (start_ns < 0) {
    metrics.message = "clock_gettime(CLOCK_MONOTONIC) failed";
    finish_with_stop_and_end(metrics, communication_type);
    return metrics;
  }

  int64_t previous_send_ns = -1;
  int64_t stream_start_ros_ns = 0;
  int64_t last_feedback_ns = start_ns - feedback_period_ns;
  int64_t scheduled_deadline_ns = start_ns;
  int64_t chosen_interval_ns = period_ns;
  uint64_t pacing_sequence = 0;
  ServoJCallTiming last_call_timing{};
  for (uint64_t sample = 0; ; ++sample) {
    ServoJTraceSample trace;
    trace.sample_index = sample;
    const bool tracing = !trace_.empty();
    trace.loop_begin_ns = tracing ? clock_.now_nanoseconds() : 0;
    auto record_sample = [&]() {
        trace.loop_end_ns = tracing ? clock_.now_nanoseconds() : 0;
        record_trace(trace);
      };
    std::array<double, 6> joints{};
    uint64_t sample_revision = 0;
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      metrics.samples_requested = buffer_offset_ + buffer_->size() / 6;
      if (sample >= metrics.samples_requested) {
        sealed_ = true;  // Keep motion ownership through end-of-stream cleanup.
        break;
      }
      std::copy_n(buffer_->begin() + (sample - buffer_offset_) * 6, 6, joints.begin());
      sample_revision = revision_;
      next_sample_ = sample + 1;
    }
    trace.buffer_ready_ns = tracing ? clock_.now_nanoseconds() : 0;
    trace.revision = sample_revision;
    trace.joints_deg = joints;
    if (cancel_requested_.load()) {
      metrics.cancelled = true;
      metrics.message = "ServoJ stream cancelled";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }

    if (queue_enabled) {
      const auto failure = read_queue();
      if (failure) {
        metrics.message = failure;
        finish_with_stop_and_end(metrics, communication_type);
        return metrics;
      }
      if (queue_snapshot.queue_length >= config_.queue.target_queue) {metrics.queue_primed = true;}
      if (sample == config_.queue.prefix_samples &&
        (!metrics.queue_primed || queue_snapshot.queue_length < config_.queue.target_queue - 1))
      {
        metrics.message = "Stationary prefix ended before queue was ready";
        finish_with_stop_and_end(metrics, communication_type);
        return metrics;
      }
      if (!metrics.queue_primed) {
        chosen_interval_ns = 4000000LL;
      } else if (queue_snapshot.sequence != pacing_sequence) {
        const int deviation = queue_snapshot.queue_length - config_.queue.target_queue;
        chosen_interval_ns = period_ns + (std::abs(deviation) <= 1 ? 0 :
          std::clamp<int64_t>(static_cast<int64_t>(deviation) * 20000LL, -60000LL, 60000LL));
      }
      pacing_sequence = queue_snapshot.sequence;
      trace.queue_phase = metrics.queue_primed ? 2 : 1;
      trace.queue_length = queue_snapshot.queue_length;
      trace.queue_received_mono_ns = queue_snapshot.received_mono_ns;
      trace.queue_sequence = queue_snapshot.sequence;
      trace.queue_age_ns = clock_.now_nanoseconds() - queue_snapshot.received_mono_ns;
      trace.scheduled_interval_ns = chosen_interval_ns;
      if (sample != 0) {scheduled_deadline_ns += chosen_interval_ns;}
    }
    const int64_t deadline_ns = queue_enabled ? scheduled_deadline_ns :
      start_ns + static_cast<int64_t>(sample) * period_ns;
    trace.deadline_ns = deadline_ns;
    trace.sleep_enter_ns = tracing ? clock_.now_nanoseconds() : 0;
    const int sleep_error = clock_.sleep_until(deadline_ns);
    if (sleep_error != 0) {
      record_sample();
      metrics.message = "clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME) failed";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    const int64_t wake_ns = clock_.now_nanoseconds();
    trace.wake_ns = wake_ns;
    if (wake_ns < 0) {
      record_sample();
      metrics.message = "clock_gettime(CLOCK_MONOTONIC) failed";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    const int64_t wake_lateness_ns = wake_ns - deadline_ns;
    const bool deadline_missed_before_call = wake_lateness_ns > tolerance_ns;
    if (deadline_missed_before_call) {
      ++metrics.missed_deadlines;
    }
    if (wake_lateness_ns > fatal_lateness_ns ||
      (queue_enabled && wake_lateness_ns >= chosen_interval_ns)) {
      record_sample();
      metrics.message = format_timing_failure(
        "ServoJ timing failure: command deadline exceeded maximum lateness",
        metrics, sample, request.period_sec, wake_lateness_ns, true, last_call_timing);
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    if (queue_enabled) {
      const auto failure = read_queue();
      if (failure || (sample == config_.queue.prefix_samples &&
        queue_snapshot.queue_length < config_.queue.target_queue - 1)) {
        record_sample();
        metrics.message = failure ? failure : "Queue lost readiness before first moving sample";
        finish_with_stop_and_end(metrics, communication_type);
        return metrics;
      }
    }
    if (cancel_requested_.load()) {
      metrics.cancelled = true;
      metrics.message = "ServoJ stream cancelled";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }

    const int command_id = allocate_command_id();
    if (command_id < 0) {
      record_sample();
      metrics.message = "ServoJ command ID exhausted the SDK integer range";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    trace.command_id = command_id;
    trace.sdk_enter_ns = tracing ? clock_.now_nanoseconds() : 0;
    error = robot_.servo_j(
      joints, static_cast<float>(sdk_cmdt_sec), command_id, communication_type);
    trace.sdk_return_ns = tracing ? clock_.now_nanoseconds() : 0;
    trace.sdk_error = error;
    trace.timing = robot_.last_servo_j_timing();
    if (error != 0) {
      trace.return_ns = clock_.now_nanoseconds();
      record_sample();
      metrics.controller_error = error;
      metrics.cancelled = cancel_requested_.load();
      metrics.message = "ServoJ failed";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    if (metrics.samples_sent == 0) {
      // Preserve the first successful SDK-return origin; this is not wire-send time.
      stream_start_ros_ns = ros_now ? ros_now() : 0;
      trace_start_ros_ns_ = stream_start_ros_ns;
    }
    ++metrics.samples_sent;
    metrics.last_command_id = static_cast<uint64_t>(command_id);
    last_call_timing = trace.timing;

    // Measure after the SDK call so mutex contention and SDK/UDP call time are reflected.
    const int64_t sent_ns = clock_.now_nanoseconds();
    trace.return_ns = sent_ns;
    if (sent_ns < 0) {
      record_sample();
      metrics.message = "clock_gettime(CLOCK_MONOTONIC) failed";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    const int64_t sent_lateness_ns = sent_ns - deadline_ns;
    if (!deadline_missed_before_call && sent_lateness_ns > tolerance_ns) {
      ++metrics.missed_deadlines;
    }
    if (previous_send_ns >= 0) {
      metrics.maximum_interval_sec = std::max(
        metrics.maximum_interval_sec,
        static_cast<double>(sent_ns - previous_send_ns) / 1.0e9);
    }
    previous_send_ns = sent_ns;
    if (sent_lateness_ns > fatal_lateness_ns ||
      (queue_enabled && sent_lateness_ns >= chosen_interval_ns)) {
      record_sample();
      metrics.message = format_timing_failure(
        "ServoJ timing failure: SDK command completed after maximum lateness",
        metrics, sample, request.period_sec, wake_lateness_ns, false, last_call_timing);
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    if (feedback &&
      (sent_ns - last_feedback_ns >= feedback_period_ns ||
      metrics.samples_sent == metrics.samples_requested))
    {
      trace.feedback_begin_ns = tracing ? clock_.now_nanoseconds() : 0;
      feedback(metrics.samples_sent, metrics.last_command_id, stream_start_ros_ns);
      trace.feedback_end_ns = tracing ? clock_.now_nanoseconds() : 0;
      last_feedback_ns = sent_ns;
    }
    record_sample();
  }

  if (cancel_requested_.load()) {
    metrics.cancelled = true;
    metrics.message = "ServoJ stream cancelled";
    finish_with_stop_and_end(metrics, communication_type);
    return metrics;
  }

  if (queue_enabled) {
    const int64_t drain_start = clock_.now_nanoseconds();
    uint64_t zero_sequence = 0;
    unsigned zero_frames = 0;
    for (;;) {
      if (cancel_requested_.load()) {
        metrics.cancelled = true;
        metrics.message = "ServoJ stream cancelled during queue drain";
        finish_with_stop_and_end(metrics, communication_type);
        return metrics;
      }
      const auto failure = read_queue();
      if (failure) {
        metrics.message = failure;
        finish_with_stop_and_end(metrics, communication_type);
        return metrics;
      }
      const int64_t now = clock_.now_nanoseconds();
      metrics.queue_drain_ns = now - drain_start;
      if (queue_snapshot.received_mono_ns > previous_send_ns && queue_snapshot.sequence != zero_sequence) {
        zero_sequence = queue_snapshot.sequence;
        zero_frames = queue_snapshot.queue_length == 0 ? zero_frames + 1 : 0;
        metrics.queue_drain_frames = zero_frames;
        if (zero_frames >= 2) {break;}
      }
      if (now < drain_start || now - drain_start >= 1000000000LL) {
        metrics.message = "Queue drain exceeded 1 second";
        finish_with_stop_and_end(metrics, communication_type);
        return metrics;
      }
      if (clock_.sleep_until(now + 2000000LL) != 0) {
        metrics.message = "Queue drain clock_nanosleep failed";
        finish_with_stop_and_end(metrics, communication_type);
        return metrics;
      }
    }
    if (cancel_requested_.load()) {
      metrics.cancelled = true;
      metrics.message = "ServoJ stream cancelled during queue drain";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
  }

  error = robot_.servo_move_end(communication_type);
  if (error != 0) {
    metrics.controller_error = error;
    metrics.cancelled = cancel_requested_.load();
    metrics.message = "ServoMoveEnd failed";
    finish_with_stop_and_end(metrics, communication_type);
    return metrics;
  }
  metrics.success = true;
  metrics.message = "ServoJ stream completed";
  return metrics;
}

void ServoJStreamer::record_trace(const ServoJTraceSample & sample)
{
  if (trace_.empty()) {return;}
  trace_[trace_count_ % trace_.size()] = sample;
  ++trace_count_;
}

void ServoJStreamer::write_trace(
  std::ostream & out, const ServoJStreamRequest & request,
  const ServoJStreamMetrics & metrics, bool include_samples) const
{
  out << std::setprecision(17);
  out << "{\"event\":\"servo_stream_result\",\"schema\":2,\"goal_id\":";
  json_string(out, request.goal_id);
  out << ",\"period_sec\":" << request.period_sec <<
    ",\"sdk_cmdt_sec\":" << metrics.sdk_cmdt_sec <<
    ",\"sdk_cmdt_float_sec\":" << static_cast<double>(static_cast<float>(metrics.sdk_cmdt_sec)) <<
    ",\"sdk_cmdt_override_active\":" << (config_.sdk_cmdt_override_sec > 0.0 ? "true" : "false") <<
    ",\"communication_type\":" << static_cast<int>(request.communication_type) <<
    ",\"stream_start_ros_ns\":" << trace_start_ros_ns_ <<
    ",\"deadline_origin_mono_ns\":" << trace_start_mono_ns_ <<
    ",\"success\":" << (metrics.success ? "true" : "false") <<
    ",\"cancelled\":" << (metrics.cancelled ? "true" : "false") <<
    ",\"controller_error\":" << metrics.controller_error <<
    ",\"samples_requested\":" << metrics.samples_requested <<
    ",\"samples_sent\":" << metrics.samples_sent <<
    ",\"last_command_id\":" << metrics.last_command_id <<
    ",\"missed_deadlines\":" << metrics.missed_deadlines <<
    ",\"queue_regulation_enabled\":" << (config_.queue.enabled ? "true" : "false") <<
    ",\"queue_prefix_samples\":" << config_.queue.prefix_samples <<
    ",\"queue_target\":" << config_.queue.target_queue <<
    ",\"queue_ceiling\":" << config_.queue.maximum_queue <<
    ",\"queue_primed\":" << (metrics.queue_primed ? "true" : "false") <<
    ",\"queue_drain_frames\":" << metrics.queue_drain_frames <<
    ",\"queue_drain_ns\":" << metrics.queue_drain_ns <<
    ",\"trace_capacity\":" << trace_.size() <<
    ",\"trace_records_total\":" << trace_count_ <<
    ",\"trace_records_overwritten\":" << (trace_count_ > trace_.size() ?
  trace_count_ - trace_.size() : 0) << ",\"message\":";
  json_string(out, metrics.message);
  out << ",\"servo_start_filter_time_ms\":" << config_.servo_start.filter_time_ms <<
    ",\"servo_start_joint_gains\":[";
  for (size_t index = 0; index < config_.servo_start.joint_gains.size(); ++index) {
    if (index != 0) {out << ',';}
    out << config_.servo_start.joint_gains[index];
  }
  const auto & feedback = metrics.feedback_publication;
  out << "],\"action_feedback\":{\"published\":" << feedback.published <<
    ",\"coalesced\":" << feedback.coalesced <<
    ",\"contended\":" << feedback.contended <<
    ",\"exceptions\":" << feedback.exceptions <<
    ",\"publication_begin_mono_ns\":" << feedback.publication_begin_ns <<
    ",\"publication_end_mono_ns\":" << feedback.publication_end_ns <<
    ",\"maximum_publication_ns\":" << feedback.maximum_publication_ns <<
    ",\"in_flight_at_close\":" <<
    (feedback.in_flight_at_close ? "true" : "false") <<
    "},\"udp_replies\":";
  robot_.write_udp_summary(out);
  out << "}\n";
  if (!include_samples || trace_.empty()) {return;}
  const uint64_t first = trace_count_ > trace_.size() ? trace_count_ - trace_.size() : 0;
  for (uint64_t index = first; index < trace_count_; ++index) {
    const auto & sample = trace_[index % trace_.size()];
    out << "{\"event\":\"servo_sample\",\"sample_index\":" << sample.sample_index <<
      ",\"buffer_revision\":" << sample.revision <<
      ",\"command_id\":" << sample.command_id <<
      ",\"sdk_attempted\":" << (sample.command_id >= 0 ? "true" : "false") <<
      ",\"sdk_error\":" << sample.sdk_error <<
      ",\"deadline_mono_ns\":" << sample.deadline_ns <<
      ",\"wake_mono_ns\":" << sample.wake_ns <<
      ",\"return_mono_ns\":" << sample.return_ns <<
      ",\"lock_wait_ns\":" << sample.timing.lock_wait_nanoseconds <<
      ",\"sdk_call_ns\":" << sample.timing.sdk_call_nanoseconds <<
      ",\"loop_begin_mono_ns\":" << sample.loop_begin_ns <<
      ",\"buffer_ready_mono_ns\":" << sample.buffer_ready_ns <<
      ",\"sleep_enter_mono_ns\":" << sample.sleep_enter_ns <<
      ",\"sdk_enter_mono_ns\":" << sample.sdk_enter_ns <<
      ",\"sdk_return_mono_ns\":" << sample.sdk_return_ns <<
      ",\"feedback_handoff_begin_mono_ns\":" << sample.feedback_begin_ns <<
      ",\"feedback_handoff_end_mono_ns\":" << sample.feedback_end_ns <<
      ",\"loop_end_mono_ns\":" << sample.loop_end_ns <<
      ",\"queue_phase\":" << sample.queue_phase <<
      ",\"queue_length\":" << sample.queue_length <<
      ",\"queue_received_mono_ns\":" << sample.queue_received_mono_ns <<
      ",\"queue_sequence\":" << sample.queue_sequence <<
      ",\"queue_age_ns\":" << sample.queue_age_ns <<
      ",\"scheduled_interval_ns\":" << sample.scheduled_interval_ns <<
      ",\"joints_deg\":";
    json_six(out, sample.joints_deg);
    out << "}\n";
  }
  robot_.write_udp_history(out);
}

int ServoJStreamer::cancel_and_stop()
{
  cancel_requested_.store(true);
  return active_.load() ? stop_once() : 0;
}

void ServoJStreamer::request_cancel()
{
  cancel_requested_.store(true);
}

bool ServoJStreamer::active() const
{
  return active_.load();
}

int ServoJStreamer::allocate_command_id()
{
  const uint64_t command_id = next_command_id_.fetch_add(1);
  return command_id <= static_cast<uint64_t>(INT_MAX) ? static_cast<int>(command_id) : -1;
}

int64_t ServoJStreamer::seconds_to_nanoseconds(double seconds)
{
  return static_cast<int64_t>(std::llround(seconds * 1.0e9));
}

std::string ServoJStreamer::format_timing_failure(
  const char * prefix, const ServoJStreamMetrics & metrics, uint64_t sample_index,
  double period_sec, int64_t wake_lateness_ns, bool before_sdk_call,
  const ServoJCallTiming & call_timing)
{
  // Keep the prefix unmodified so consumers can keep matching on the existing
  // failure strings; the measured breakdown is appended in a bracketed suffix.
  std::ostringstream message;
  message << prefix << std::fixed <<
    " [sample_index=" << sample_index <<
    " samples_sent=" << metrics.samples_sent <<
    std::setprecision(6) << " period_sec=" << period_sec <<
    std::setprecision(3) <<
    " wake_lateness_ms=" << nanoseconds_to_milliseconds(wake_lateness_ns);
  if (before_sdk_call) {
    // The prior SDK timings are context; they do not identify the cause of a late wake.
    message <<
      " sdk_call_ms=0.000 lock_wait_ms=0.000" <<
      " prev_sdk_call_ms=" <<
      nanoseconds_to_milliseconds(call_timing.sdk_call_nanoseconds) <<
      " prev_lock_wait_ms=" <<
      nanoseconds_to_milliseconds(call_timing.lock_wait_nanoseconds);
  } else {
    message <<
      " sdk_call_ms=" << nanoseconds_to_milliseconds(call_timing.sdk_call_nanoseconds) <<
      " lock_wait_ms=" << nanoseconds_to_milliseconds(call_timing.lock_wait_nanoseconds);
  }
  message <<
    " max_interval_ms=" << metrics.maximum_interval_sec * 1.0e3 <<
    " missed_deadlines=" << metrics.missed_deadlines << "]";
  return message.str();
}

}  // namespace fairino_hardware
