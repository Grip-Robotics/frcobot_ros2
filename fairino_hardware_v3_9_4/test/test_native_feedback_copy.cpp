#include "fairino_hardware/native_feedback_copy.hpp"
#include <gtest/gtest.h>
#include <array>
#include <cstring>

namespace
{
using fairino_hardware::NativeFeedbackCopy;
using Match = NativeFeedbackCopy::Match;

ROBOT_STATE_PKG make_state(double speed)
{
  ROBOT_STATE_PKG state{};
  state.frame_head = 0x5a5a;
  state.data_len = sizeof(state) - 7;
  state.frame_cnt = 32;
  state.robotTime.millisecond = 271;
  state.jt_cur_pos[4] = 77.649;
  state.actual_qd[4] = speed;
  state.mc_queue_len = 19;
  const auto * bytes = reinterpret_cast<const uint8_t *>(&state);
  for (size_t i = 0; i < offsetof(ROBOT_STATE_PKG, check_sum); ++i) {state.check_sum += bytes[i];}
  return state;
}

auto packet(const ROBOT_STATE_PKG & state)
{
  std::array<uint8_t, 40 + sizeof(state)> bytes{};
  const auto size = bytes.size();
  bytes[0] = 0x45;
  bytes[2] = size >> 8;
  bytes[3] = size & 255;
  bytes[9] = 6;
  bytes[20] = 20004 >> 8;
  bytes[21] = 20004 & 255;
  bytes[32] = 0x50;
  bytes[33] = 0x18;
  memcpy(bytes.data() + 40, &state, sizeof(state));
  uint32_t sum = 6 + size - 20;
  for (size_t i = 20; i < size; i += 2) {sum += (bytes[i] << 8) + bytes[i + 1];}
  while (sum >> 16) {sum = (sum >> 16) + (sum & 65535);}
  bytes[36] = (~sum >> 8) & 255;
  bytes[37] = ~sum & 255;
  return bytes;
}

TEST(NativeFeedbackCopy, CorrectsBothDirectionsAndPreservesAControllerOverspeed)
{
  for (auto speed : {-20.0, -208.631}) {
    NativeFeedbackCopy copy;
    auto sdk = make_state(speed);
    const auto p = packet(sdk);
    copy.receive(p.data(), p.size(), 1000000000);
    EXPECT_EQ(copy.reconcile(sdk, 1000000001), Match::Identical);
    sdk.actual_qd[4] = speed == -20.0 ? -208.631 : -20.0;
    EXPECT_EQ(copy.reconcile(sdk, 1000000001), Match::Repaired);
    EXPECT_EQ(sdk.actual_qd[4], speed);
    EXPECT_EQ(copy.reconcile(sdk, 1000000001), Match::Identical);
  }
}

TEST(NativeFeedbackCopy, IgnoresUnmatchedStaleAndDamagedPackets)
{
  NativeFeedbackCopy copy;
  auto sdk = make_state(-20.0);
  auto p = packet(sdk);
  copy.receive(p.data(), p.size(), 1000000000);
  sdk.actual_qd[4] = -208.631;
  EXPECT_EQ(copy.reconcile(sdk, 1030000001), Match::Unavailable);
  ++sdk.frame_cnt;
  EXPECT_EQ(copy.reconcile(sdk, 1000000001), Match::Unavailable);
  --sdk.frame_cnt;
  sdk.main_code = 4;
  EXPECT_EQ(copy.reconcile(sdk, 1000000001), Match::Unavailable);
  sdk.main_code = 0;
  NativeFeedbackCopy invalid;
  p.back() ^= 1;
  invalid.receive(p.data(), p.size(), 1000000000);
  p = packet(sdk);  // Carrier checksum is valid; native checksum is stale.
  invalid.receive(p.data(), p.size(), 1000000000);
  invalid.receive(p.data(), p.size() - 1, 1000000000);
  EXPECT_EQ(invalid.reconcile(sdk, 1000000001), Match::Unavailable);
  EXPECT_EQ(sdk.actual_qd[4], -208.631);
}

TEST(NativeFeedbackCopy, RefusesTwoDifferentOriginalsWithTheSameControllerChecksum)
{
  NativeFeedbackCopy copy;
  auto one = make_state(-20.0);
  auto two = one;
  std::swap(two.actual_qd[3], two.actual_qd[4]);
  for (const auto & state : {one, two}) {
    auto p = packet(state);
    copy.receive(p.data(), p.size(), 1000000000);
  }
  one.actual_qd[4] = -208.631;
  EXPECT_EQ(copy.reconcile(one, 1000000001), Match::Unavailable);
}
}
