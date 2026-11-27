#pragma once

#include <array>
#include <cstdint>
#include <string>
#include "libfairino/include/robot_types.h"

namespace fairino_hardware
{
// Independent receive-only copy of feedback on the SDK's existing IPv4 connection.
class NativeFeedbackCopy
{
public:
  NativeFeedbackCopy() = default;
  ~NativeFeedbackCopy();
  NativeFeedbackCopy(const NativeFeedbackCopy &) = delete;
  NativeFeedbackCopy & operator=(const NativeFeedbackCopy &) = delete;

  void poll(const std::string & controller, int64_t now_ns);
  enum class Match {Unavailable, Identical, Repaired};
  Match reconcile(ROBOT_STATE_PKG & sdk, int64_t now_ns) const;

  // Accept only one complete native state carried by a TCP segment of the filtered flow.
  void receive(const uint8_t * ip, size_t size, int64_t received_ns);

private:
  void close_receiver();
  bool attach(const std::string & controller);
  static constexpr int64_t max_age_ns = 30000000;
  struct Entry {ROBOT_STATE_PKG state{}; int64_t received_ns{};};
  std::array<Entry, 8> history_{};
  size_t next_{};
  int fd_{-1}, sdk_fd_{-1};
  uint64_t sdk_cookie_{};
  int64_t next_attach_ns_{};
};
}  // namespace fairino_hardware
