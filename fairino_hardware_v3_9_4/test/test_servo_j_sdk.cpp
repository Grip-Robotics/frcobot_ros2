#include "libfairino/include/robot.h"
#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace
{
std::vector<std::string> frames;
}

extern "C" int capture_servo_j_udp(void *, std::string frame)
  __asm__("_ZN11FRUdpClient9SendFrameENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE");

extern "C" int capture_servo_j_udp(void *, std::string frame)
{
  frames.push_back(std::move(frame));
  return 0;
}

TEST(ServoJSdk, PinnedLibrarySerializesSameTimedSendWithoutNetworking)
{
#ifdef FAIRINO_SDK_234
  FRRobot sdk;
  const char * version_token = "2.3.4";
#else
  FRRobot sdk(false);
  const char * version_token = "2.4.1";
#endif
  char version[128]{};
  ASSERT_EQ(sdk.GetSDKVersion(version), 0);
  EXPECT_NE(std::string(version).find(version_token), std::string::npos);
  JointPos joints;
  const double targets[] = {1.25, -2.5, 3.75, 4.5, -5.25, 6.0};
  for (size_t index = 0; index < 6; ++index) {joints.jPos[index] = targets[index];}
  ExaxisPos external;
  for (double & position : external.ePos) {position = 0;}
  frames.clear();
  ASSERT_EQ(sdk.ServoJ(&joints, &external, 0, 0, 0.005F, 0, 0, 42, 1), 0);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_NE(frames[0].find("III376III"), std::string::npos);
  EXPECT_NE(frames[0].find("1.250,-2.500,3.750,4.500,-5.250,6.000"), std::string::npos);
  EXPECT_NE(frames[0].find("0.005"), std::string::npos);
  EXPECT_NE(frames[0].find("42"), std::string::npos);
}
