#include "fairino_hardware/servo_start_settings.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace
{
struct RecordingSdk
{
  int ServoMoveStart(int communication_type, int filter_time_ms, std::vector<double> gains)
  {
    ++calls;
    transport = communication_type;
    filter = filter_time_ms;
    joint_gains = gains;
    return result;
  }

  int calls{0};
  int transport{-1};
  int filter{-1};
  int result{0};
  std::vector<double> joint_gains;
};

TEST(ServoStartSettings, PassesDistinctJointGainsAndMillisecondsThroughBothTransports)
{
  fairino_hardware::ServoStartSettings settings;
  settings.filter_time_ms = 17;
  settings.joint_gains = {11.125, 12.25, -1.0, 14.5, 15.75, 16.875};
  for (const int transport : {0, 1}) {
    RecordingSdk sdk;
    sdk.result = 42;
    EXPECT_EQ(fairino_hardware::start_servo_with_settings(sdk, transport, settings), 42);
    EXPECT_EQ(sdk.calls, 1);
    EXPECT_EQ(sdk.transport, transport);
    EXPECT_EQ(sdk.filter, 17);
    EXPECT_EQ(sdk.joint_gains, settings.joint_gains);
  }
}

TEST(ServoStartSettings, DefaultsRequestZeroFilterAndControllerGains)
{
  RecordingSdk sdk;
  EXPECT_EQ(fairino_hardware::start_servo_with_settings(
      sdk, 1, fairino_hardware::ServoStartSettings{}), 0);
  EXPECT_EQ(sdk.filter, 0);
  EXPECT_EQ(sdk.joint_gains, std::vector<double>(6, -1.0));
}

TEST(ServoStartSettings, RejectsInvalidSettingsBeforeCallingSdk)
{
  std::vector<fairino_hardware::ServoStartSettings> invalid;
  auto settings = fairino_hardware::ServoStartSettings{};
  settings.filter_time_ms = -1;
  invalid.push_back(settings);
  settings.filter_time_ms = static_cast<int64_t>(std::numeric_limits<int>::max()) + 1;
  invalid.push_back(settings);
  for (const size_t length : {0U, 5U, 7U}) {
    settings = {};
    settings.joint_gains.resize(length);
    invalid.push_back(settings);
  }
  for (const double gain : {-0.5, -2.0, std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::max()})
  {
    settings = {};
    settings.joint_gains[3] = gain;
    invalid.push_back(settings);
  }
  RecordingSdk sdk;
  for (const auto & value : invalid) {
    EXPECT_THROW(fairino_hardware::start_servo_with_settings(sdk, 1, value),
      std::invalid_argument);
  }
  EXPECT_THROW(fairino_hardware::start_servo_with_settings(sdk, 2, {}),
    std::invalid_argument);
  EXPECT_EQ(sdk.calls, 0);
}
}  // namespace
