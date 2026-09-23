#include "fairino_hardware/gripper_command.hpp"

#include <gtest/gtest.h>
#include <limits>

using fairino_hardware::GripperMove;
using fairino_hardware::parse_gripper_activation;
using fairino_hardware::parse_gripper_move;

namespace
{
const GripperMove defaults{0, 0, 80, 50, 1234, 0};
}

TEST(GripperCommand, TwoArgumentsHonorConfiguredTimeoutAndBlockingMode)
{
  const auto command = parse_gripper_move("2,100", defaults);
  EXPECT_EQ(command.index, 2);
  EXPECT_EQ(command.position, 100);
  EXPECT_EQ(command.speed, 80);
  EXPECT_EQ(command.force, 50);
  EXPECT_EQ(command.max_time_ms, 1234);
  EXPECT_EQ(command.block, 0);
}

TEST(GripperCommand, ExplicitJodellCommandKeepsFullSpeedForceAndNonBlockingMode)
{
  const auto command = parse_gripper_move("2,0,100,100,3000,1", defaults);
  EXPECT_EQ(command.index, 2);
  EXPECT_EQ(command.position, 0);
  EXPECT_EQ(command.speed, 100);
  EXPECT_EQ(command.force, 100);
  EXPECT_EQ(command.max_time_ms, 3000);
  EXPECT_EQ(command.block, 1);
}

TEST(GripperCommand, FourArgumentsPreserveSpeedForceOverrides)
{
  const auto command = parse_gripper_move("2,100,90,100", defaults);
  EXPECT_EQ(command.speed, 90);
  EXPECT_EQ(command.force, 100);
  EXPECT_EQ(command.max_time_ms, 1234);
  EXPECT_EQ(command.block, 0);
}

TEST(GripperCommand, RejectsIncompleteAndMalformedCommands)
{
  for (const auto * args : {"", "2", "2,", "2,100,50", "2,100,50,50,3000",
      "2,100,50,50,3000,1,0", "2,10oops", "2,1.0", "2,,", "2,100,",
      "2,999999999999999999999"})
  {
    EXPECT_THROW(parse_gripper_move(args, defaults), std::invalid_argument) << args;
  }
}

TEST(GripperCommand, RejectsOutOfRangeMotionBeforeDispatch)
{
  for (const auto * args : {"0,100", "9,0", "2,-1", "2,101", "2,0,-1,100",
      "2,0,100,101", "2,0,101,100", "2,0,100,-1", "2,0,100,100,-1,1",
      "2,0,100,100,30001,1", "2,0,100,100,3000,256", "2,0,100,100,3000,-1"})
  {
    EXPECT_THROW(parse_gripper_move(args, defaults), std::out_of_range) << args;
  }
}

TEST(GripperCommand, RejectsInvalidConfiguredDefaults)
{
  EXPECT_THROW(parse_gripper_move("2,0", (GripperMove{0, 0, 80, 50, 30001, 1})),
    std::out_of_range);
  EXPECT_THROW(parse_gripper_move("2,0", (GripperMove{0, 0, 80, 50, 3000, 2})),
    std::out_of_range);
  EXPECT_THROW(parse_gripper_move("2,0", (GripperMove{0, 0, 80, 50,
    std::numeric_limits<int64_t>::max(), 1})), std::out_of_range);
}

TEST(GripperCommand, PreservesIntegerWhitespaceAndPositiveSigns)
{
  const auto command = parse_gripper_move(" +2 , 100 , 100, 100 , 3000 , +1 ", defaults);
  EXPECT_EQ(command.index, 2);
  EXPECT_EQ(command.max_time_ms, 3000);
  EXPECT_EQ(command.block, 1);
  EXPECT_THROW(parse_gripper_move("2, + ", defaults), std::invalid_argument);
  EXPECT_THROW(parse_gripper_move("2, +-0", defaults), std::invalid_argument);
}

TEST(GripperCommand, ActivationValidatesBeforeNarrowingToByte)
{
  EXPECT_EQ(parse_gripper_activation("2,1"), (std::array<int, 2>{2, 1}));
  EXPECT_EQ(parse_gripper_activation("2,0"), (std::array<int, 2>{2, 0}));
  for (const auto * args : {"", "2", "2,1,0", "2,1junk", "2,+-0"}) {
    EXPECT_THROW(parse_gripper_activation(args), std::invalid_argument);
  }
  for (const auto * args : {"0,1", "9,1", "2,256", "2,-1"}) {
    EXPECT_THROW(parse_gripper_activation(args), std::out_of_range);
  }
}

TEST(GripperCommand, NativeCommandsCannotEnterSdkDuringEitherServoSession)
{
  for (const auto * command : {"ActGripper", "MoveGripper"}) {
    EXPECT_FALSE(fairino_hardware::gripper_conflicts_with_servo(command, false, false));
    EXPECT_TRUE(fairino_hardware::gripper_conflicts_with_servo(command, true, false));
    EXPECT_TRUE(fairino_hardware::gripper_conflicts_with_servo(command, false, true));
    EXPECT_TRUE(fairino_hardware::gripper_conflicts_with_servo(command, true, true));
  }
  // Emergency stop and ending a legacy session must remain dispatchable.
  EXPECT_FALSE(fairino_hardware::gripper_conflicts_with_servo("StopMotion", true, true));
  EXPECT_FALSE(fairino_hardware::gripper_conflicts_with_servo("ServoMoveEnd", false, true));
  EXPECT_FALSE(fairino_hardware::gripper_conflicts_with_servo("ServoJ", false, true));
}
