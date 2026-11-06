#include "fairino_hardware/controller_state_progress.hpp"
#include <gtest/gtest.h>

namespace
{
struct State
{
  uint8_t frame_cnt{0};
  struct Time
  {
    uint16_t year{0}, mouth{0}, day{0}, hour{0}, minute{0}, second{0}, millisecond{0};
  } robotTime;
};

State valid()
{
  State state;
  state.robotTime.year = 2026;
  return state;
}

TEST(ControllerStateProgress, NeverPublishesSuccessfulZeroCacheWithoutProgress)
{
  fairino_hardware::ControllerStateProgress progress;
  for (int read = 0; read < 1000; ++read) {EXPECT_FALSE(progress.observe(State{}));}
  EXPECT_FALSE(progress.observe(valid()));  // Prime even a valid frame zero.
  auto state = valid();
  state.frame_cnt = 1;
  EXPECT_TRUE(progress.observe(state));
}

TEST(ControllerStateProgress, RepeatedCacheCannotKeepDownstreamWatchdogFresh)
{
  fairino_hardware::ControllerStateProgress progress;
  auto state = valid();
  ASSERT_FALSE(progress.observe(state));
  state.frame_cnt = 1;
  ASSERT_TRUE(progress.observe(state));
  for (int read = 0; read < 1000; ++read) {EXPECT_FALSE(progress.observe(state));}
}

TEST(ControllerStateProgress, AcceptsCounterWrapAndClockChangeAtSameCounter)
{
  fairino_hardware::ControllerStateProgress progress;
  auto state = valid();
  state.frame_cnt = 255;
  EXPECT_FALSE(progress.observe(state));
  state.frame_cnt = 0;
  EXPECT_TRUE(progress.observe(state));
  EXPECT_FALSE(progress.observe(state));
  state.robotTime.millisecond = 8;
  EXPECT_TRUE(progress.observe(state));
  state.robotTime.year = 2027;
  EXPECT_TRUE(progress.observe(state));
}

TEST(ControllerStateProgress, InvalidReadDoesNotErasePreviousIdentityAndProgressRecovers)
{
  fairino_hardware::ControllerStateProgress progress;
  auto state = valid();
  EXPECT_FALSE(progress.observe(state));
  EXPECT_FALSE(progress.observe(State{}));
  EXPECT_FALSE(progress.observe(state));
  state.frame_cnt = 1;
  EXPECT_TRUE(progress.observe(state));
}
}  // namespace
