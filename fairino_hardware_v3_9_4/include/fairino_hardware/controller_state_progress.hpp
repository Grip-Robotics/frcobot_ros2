#ifndef FAIRINO_HARDWARE__CONTROLLER_STATE_PROGRESS_HPP_
#define FAIRINO_HARDWARE__CONTROLLER_STATE_PROGRESS_HPP_

#include <array>
#include <cstdint>

namespace fairino_hardware
{
class ControllerStateProgress
{
public:
  template<typename State>
  bool observe(const State & state)
  {
    // SDK 2.4.1 synthesizes state from CNDE, so raw TCP envelope fields are not required.
    const auto & time = state.robotTime;
    const std::array<uint16_t, 7> tick{time.year, time.mouth, time.day, time.hour,
      time.minute, time.second, time.millisecond};
    if (state.frame_cnt == 0 && tick == std::array<uint16_t, 7>{}) {return false;}
    if (have_state_ && frame_ == state.frame_cnt && tick_ == tick) {return false;}
    // The first valid cache snapshot alone does not establish live progress.
    const bool progressed = have_state_;
    have_state_ = true;
    frame_ = state.frame_cnt;
    tick_ = tick;
    return progressed;
  }

private:
  bool have_state_{false};
  uint8_t frame_{0};
  std::array<uint16_t, 7> tick_{};
};
}  // namespace fairino_hardware

#endif  // FAIRINO_HARDWARE__CONTROLLER_STATE_PROGRESS_HPP_
