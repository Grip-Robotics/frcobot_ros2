#ifndef FAIRINO_HARDWARE__SERVO_START_SETTINGS_HPP_
#define FAIRINO_HARDWARE__SERVO_START_SETTINGS_HPP_

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace fairino_hardware
{

// SDK 2.4.1 / controller 4.0.1 ServoMoveStart settings. These are distinct
// from the unavailable filterT/gain arguments on each ServoJ sample.
struct ServoStartSettings
{
  int64_t filter_time_ms{0};
  std::vector<double> joint_gains{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0};

  bool validate(std::string & reason) const
  {
    if (filter_time_ms < 0 || filter_time_ms > std::numeric_limits<int>::max()) {
      reason = "servo_j.filter_time_ms must be a non-negative SDK integer in milliseconds";
      return false;
    }
    if (joint_gains.size() != 6) {
      reason = "servo_j.joint_gains must contain exactly six values in J1..J6 order";
      return false;
    }
    for (const double gain : joint_gains) {
      if (!std::isfinite(gain) || (gain < 0.0 && gain != -1.0)) {
        reason = "servo_j.joint_gains must be finite and either -1 (default) or non-negative";
        return false;
      }
    }
    // The vendor's UDP implementation formats these arguments into 128 bytes.
    // Reject settings it would silently truncate, without inventing tuning limits.
    const int length = std::snprintf(nullptr, 0, "%d,{%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}",
      static_cast<int>(filter_time_ms), joint_gains[0], joint_gains[1], joint_gains[2],
      joint_gains[3], joint_gains[4], joint_gains[5]);
    if (length < 0 || length >= 128) {
      reason = "servo_j settings exceed the SDK's ServoMoveStart UDP argument buffer";
      return false;
    }
    return true;
  }
};

// Shared by the buffered action and legacy string-command paths. Keeping the
// SDK boundary here allows argument ordering and errors to be tested offline.
template<typename Robot>
int start_servo_with_settings(Robot & robot, int communication_type,
  const ServoStartSettings & settings)
{
  std::string reason;
  if (!settings.validate(reason)) {
    throw std::invalid_argument(reason);
  }
  if (communication_type != 0 && communication_type != 1) {
    throw std::invalid_argument("ServoMoveStart communication type must be 0 or 1");
  }
  return robot.ServoMoveStart(communication_type,
    static_cast<int>(settings.filter_time_ms), settings.joint_gains);
}

}  // namespace fairino_hardware

#endif  // FAIRINO_HARDWARE__SERVO_START_SETTINGS_HPP_
