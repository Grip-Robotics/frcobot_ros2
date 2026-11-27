#pragma once

#include <cstddef>
#include <cstdint>

namespace fairino_hardware
{
// SDK 2.3.4 copies the packed TCP/20004 frame, including its trailing checksum.
template<typename State>
bool valid_feedback_envelope(const State & state)
{
  static_assert(offsetof(State, check_sum) + sizeof(state.check_sum) == sizeof(State));
  if (state.frame_head != 0x5a5a || size_t(state.data_len) + 7 != sizeof(State)) {return false;}
  const auto * bytes = reinterpret_cast<const unsigned char *>(&state);
  uint32_t sum = 0;
  for (size_t i = 0; i < offsetof(State, check_sum); ++i) {sum += bytes[i];}
  return static_cast<uint16_t>(sum) == state.check_sum;
}
}  // namespace fairino_hardware
