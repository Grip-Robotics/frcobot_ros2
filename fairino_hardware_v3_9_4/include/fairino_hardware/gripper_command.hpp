#pragma once

#include <array>
#include <charconv>
#include <cctype>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace fairino_hardware
{

struct GripperMove
{
  int index;
  int position;
  int64_t speed;
  int64_t force;
  int64_t max_time_ms;
  int64_t block;
};

inline std::vector<int> gripper_arguments(const std::string & arguments)
{
  std::vector<int> values;
  size_t start = 0;
  do {
    const auto comma = arguments.find(',', start);
    const auto end = comma == std::string::npos ? arguments.size() : comma;
    auto first = arguments.data() + start;
    auto last = arguments.data() + end;
    while (first != last && std::isspace(static_cast<unsigned char>(*first))) {++first;}
    while (first != last && std::isspace(static_cast<unsigned char>(last[-1]))) {--last;}
    if (first != last && *first == '+') {
      ++first;
      if (first == last || *first < '0' || *first > '9') {
        throw std::invalid_argument("Gripper arguments must be complete integers");
      }
    }
    int value = 0;
    const auto parsed = std::from_chars(first, last, value);
    if (parsed.ec != std::errc{} || parsed.ptr != last) {
      throw std::invalid_argument("Gripper arguments must be complete integers");
    }
    values.push_back(value);
    if (comma == std::string::npos) {
      break;
    }
    start = comma + 1;
  } while (true);
  return values;
}

inline void gripper_range(int64_t value, int minimum, int maximum)
{
  if (value < minimum || value > maximum) {
    throw std::out_of_range("Gripper argument outside the supported range");
  }
}

inline GripperMove parse_gripper_move(const std::string & arguments, GripperMove defaults)
{
  const auto values = gripper_arguments(arguments);
  // Preserve the documented two-argument form and the previously accepted
  // four-argument speed/force override. Six arguments also override wait mode.
  if (values.size() != 2 && values.size() != 4 && values.size() != 6) {
    throw std::invalid_argument("MoveGripper expects 2, 4 or 6 arguments");
  }
  defaults.index = values[0];
  defaults.position = values[1];
  if (values.size() >= 4) {
    defaults.speed = values[2];
    defaults.force = values[3];
  }
  if (values.size() == 6) {
    defaults.max_time_ms = values[4];
    defaults.block = values[5];
  }
  gripper_range(defaults.index, 1, 8);
  gripper_range(defaults.position, 0, 100);
  gripper_range(defaults.speed, 0, 100);
  gripper_range(defaults.force, 0, 100);
  gripper_range(defaults.max_time_ms, 0, 30000);
  gripper_range(defaults.block, 0, 1);
  return defaults;
}

inline std::array<int, 2> parse_gripper_activation(const std::string & arguments)
{
  const auto values = gripper_arguments(arguments);
  if (values.size() != 2) {
    throw std::invalid_argument("ActGripper expects 2 arguments");
  }
  gripper_range(values[0], 1, 8);
  gripper_range(values[1], 0, 1);
  return {values[0], values[1]};
}

inline bool gripper_conflicts_with_servo(
  const std::string & command, bool buffered_active, bool legacy_active)
{
  return (buffered_active || legacy_active) &&
         (command == "ActGripper" || command == "MoveGripper");
}

}  // namespace fairino_hardware
