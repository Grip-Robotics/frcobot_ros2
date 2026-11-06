#include "fairino_hardware/servo_start_settings.hpp"
#include "libfairino/include/robot.h"
#include "libfairino/include/robot_error.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{
std::vector<std::string> sent_frames;
int send_result = 0;
}  // namespace

// Interpose only the vendor's UDP send boundary in this test executable. The
// real SDK constructs/serializes the command, but no RPC/connect/send reaches
// a controller. This symbol is specific to the pinned Linux x86-64 SDK ABI.
extern "C" int capture_servo_udp(void *, std::string frame)
  __asm__("_ZN11FRUdpClient9SendFrameENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE");

extern "C" int capture_servo_udp(void *, std::string frame)
{
  sent_frames.push_back(frame);
  return send_result;
}

TEST(ServoStartSdk, RealLibrarySerializesFilterAndAllSixGainsWithoutConnecting)
{
  sent_frames.clear();
  send_result = 0;
  FRRobot sdk(false);
  char version[128]{};
  ASSERT_EQ(sdk.GetSDKVersion(version), 0);
  EXPECT_STREQ(version, "SDK V2.4.1.0-robot v4.0.1");
  fairino_hardware::ServoStartSettings settings;
  settings.filter_time_ms = 17;
  settings.joint_gains = {11.125, 12.25, -1.0, 14.5, 15.75, 16.875};
  ASSERT_EQ(fairino_hardware::start_servo_with_settings(sdk, 1, settings), 0);
  ASSERT_EQ(sent_frames.size(), 1U);
  EXPECT_NE(sent_frames[0].find(
      "ServoMoveStart(17,{11.125,12.250,-1.000,14.500,15.750,16.875})"),
    std::string::npos);
  EXPECT_NE(sent_frames[0].find("III689III"), std::string::npos);

  send_result = -1;
  EXPECT_EQ(fairino_hardware::start_servo_with_settings(sdk, 1, settings),
    ERR_SOCKET_SEND_FAILED);
  EXPECT_EQ(sent_frames.size(), 2U);
  send_result = 0;
}
