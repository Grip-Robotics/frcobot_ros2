#include "fairino_hardware/native_feedback_copy.hpp"
#include "fairino_hardware/feedback_envelope.hpp"

#include <arpa/inet.h>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <linux/filter.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

namespace fairino_hardware
{
namespace
{
uint16_t be16(const uint8_t * p) {return (uint16_t(p[0]) << 8) | p[1];}

uint32_t word_sum(const uint8_t * bytes, size_t size)
{
  uint32_t sum = 0;
  while (size >= 2) {sum += be16(bytes); bytes += 2; size -= 2;}
  return sum + (size ? uint32_t(bytes[0]) << 8 : 0);
}

uint64_t socket_cookie(int fd)
{
  uint64_t value{};
  socklen_t size = sizeof(value);
  return getsockopt(fd, SOL_SOCKET, SO_COOKIE, &value, &size) == 0 ? value : 0;
}

int feedback_socket(in_addr peer)
{
  DIR * dir = opendir("/proc/self/fd");
  if (!dir) {return -1;}
  int found = -1;
  while (auto * entry = readdir(dir)) {
    char * end;
    int fd = static_cast<int>(strtol(entry->d_name, &end, 10));
    if (*end) {continue;}
    sockaddr_in remote{};
    socklen_t size = sizeof(remote);
    if (getpeername(fd, reinterpret_cast<sockaddr *>(&remote), &size) != 0 ||
      remote.sin_family != AF_INET || remote.sin_addr.s_addr != peer.s_addr ||
      remote.sin_port != htons(20004)) {continue;}
    int protocol{};
    size = sizeof(protocol);
    if (getsockopt(fd, SOL_SOCKET, SO_PROTOCOL, &protocol, &size) != 0 ||
      protocol != IPPROTO_TCP) {continue;}
    if (found >= 0) {found = -1; break;}
    found = fd;
  }
  closedir(dir);
  return found;
}
}  // namespace

NativeFeedbackCopy::~NativeFeedbackCopy() {close_receiver();}

void NativeFeedbackCopy::close_receiver()
{
  if (fd_ >= 0) {close(fd_);}
  fd_ = sdk_fd_ = -1;
  sdk_cookie_ = 0;
  history_ = {};
  next_ = 0;
}

bool NativeFeedbackCopy::attach(const std::string & controller)
{
  in_addr peer{};
  if (inet_pton(AF_INET, controller.c_str(), &peer) != 1) {return false;}
  sdk_fd_ = feedback_socket(peer);
  if (sdk_fd_ < 0 || !(sdk_cookie_ = socket_cookie(sdk_fd_))) {return false;}
  sockaddr_in local{};
  socklen_t size = sizeof(local);
  if (getsockname(sdk_fd_, reinterpret_cast<sockaddr *>(&local), &size) != 0) {return false;}
  const uint16_t local_port = ntohs(local.sin_port);
  local.sin_port = 0;
  // A raw receive socket makes no connection and sends no application packets.
  fd_ = socket(AF_INET, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, IPPROTO_TCP);
  if (fd_ < 0) {return false;}
  sock_filter rules[] = {
    BPF_STMT(BPF_LD | BPF_W | BPF_ABS, 12),
    BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, ntohl(peer.s_addr), 0, 6),
    BPF_STMT(BPF_LDX | BPF_B | BPF_MSH, 0),
    BPF_STMT(BPF_LD | BPF_H | BPF_IND, 0),
    BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, 20004, 0, 3),
    BPF_STMT(BPF_LD | BPF_H | BPF_IND, 2),
    BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, local_port, 0, 1),
    BPF_STMT(BPF_RET | BPF_K, 65535),
    BPF_STMT(BPF_RET | BPF_K, 0)
  };
  sock_fprog filter{static_cast<unsigned short>(sizeof(rules) / sizeof(rules[0])), rules};
  const int on = 1;
  if (setsockopt(fd_, SOL_SOCKET, SO_ATTACH_FILTER, &filter, sizeof(filter)) ||
    setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on)) ||
    bind(fd_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) ||
    socket_cookie(sdk_fd_) != sdk_cookie_)
  {close_receiver(); return false;}
  return true;
}

void NativeFeedbackCopy::poll(const std::string & controller, int64_t now_ns)
{
  if (fd_ >= 0 && socket_cookie(sdk_fd_) != sdk_cookie_) {close_receiver();}
  if (fd_ < 0) {
    if (now_ns < next_attach_ns_) {return;}
    next_attach_ns_ = now_ns + 1000000000LL;
    if (!attach(controller)) {return;}
  }
  timespec wall{};
  if (clock_gettime(CLOCK_REALTIME, &wall)) {return;}
  const int64_t wall_ns = wall.tv_sec * 1000000000LL + wall.tv_nsec;
  // A standard 50 ms state callback can accumulate 50 one-millisecond frames.
  // Discard expired datagrams while draining toward the current SDK frame.
  for (unsigned n = 0; n < 128; ++n) {
    uint8_t data[sizeof(ROBOT_STATE_PKG) + 120];
    alignas(cmsghdr) char control[CMSG_SPACE(sizeof(timespec))];
    iovec io{data, sizeof(data)};
    msghdr msg{};
    msg.msg_iov = &io;
    msg.msg_iovlen = 1;
    msg.msg_control = control;
    msg.msg_controllen = sizeof(control);
    const auto bytes = recvmsg(fd_, &msg, MSG_DONTWAIT);
    if (bytes <= 0) {break;}
    if (msg.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) {continue;}
    for (auto * c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
      if (c->cmsg_level != SOL_SOCKET || c->cmsg_type != SCM_TIMESTAMPNS ||
        c->cmsg_len != CMSG_LEN(sizeof(timespec))) {continue;}
      timespec stamp{};
      memcpy(&stamp, CMSG_DATA(c), sizeof(stamp));
      const auto age = wall_ns - (stamp.tv_sec * 1000000000LL + stamp.tv_nsec);
      if (age >= 0 && age <= max_age_ns) {receive(data, bytes, now_ns - age);}
    }
  }
}

void NativeFeedbackCopy::receive(const uint8_t * ip, size_t size, int64_t received_ns)
{
  if (size < 40 || (ip[0] >> 4) != 4 || ip[9] != IPPROTO_TCP ||
    be16(ip + 2) != size || (be16(ip + 6) & 0x3fff)) {return;}
  const size_t ih = (ip[0] & 15) * 4;
  if (ih < 20 || ih + 20 > size) {return;}
  const auto * tcp = ip + ih;
  const size_t th = (tcp[12] >> 4) * 4;
  if (th < 20 || ih + th + sizeof(ROBOT_STATE_PKG) != size ||
    be16(tcp) != 20004 || (tcp[13] & 7)) {return;}
  uint32_t sum = word_sum(ip + 12, 8) + IPPROTO_TCP + size - ih + word_sum(tcp, size - ih);
  while (sum >> 16) {sum = (sum & 65535) + (sum >> 16);}
  if (sum != 65535) {return;}
  ROBOT_STATE_PKG state{};
  memcpy(&state, tcp + th, sizeof(state));
  if (!valid_feedback_envelope(state)) {return;}
  history_[next_] = {state, received_ns};
  next_ = (next_ + 1) % history_.size();
}

NativeFeedbackCopy::Match NativeFeedbackCopy::reconcile(ROBOT_STATE_PKG & sdk, int64_t now_ns) const
{
  const ROBOT_STATE_PKG * original = nullptr;
  const auto offset = offsetof(ROBOT_STATE_PKG, actual_qd);
  const auto end = offset + sizeof(sdk.actual_qd);
  for (const auto & entry : history_) {
    if (entry.received_ns <= 0 || entry.received_ns > now_ns ||
      now_ns - entry.received_ns > max_age_ns) {continue;}
    const auto & wire = entry.state;
    // Require identical identity, queue, positions, fault bits and every other byte,
    // including the controller's original checksum. Only the speed bytes may differ.
    if (memcmp(&sdk, &wire, offset) ||
      memcmp(reinterpret_cast<const uint8_t *>(&sdk) + end,
        reinterpret_cast<const uint8_t *>(&wire) + end, sizeof(sdk) - end)) {continue;}
    if (original && memcmp(original->actual_qd, wire.actual_qd, sizeof(wire.actual_qd))) {
      return Match::Unavailable;
    }
    original = &wire;
  }
  if (!original) {return Match::Unavailable;}
  if (!memcmp(original->actual_qd, sdk.actual_qd, sizeof(sdk.actual_qd))) {
    return Match::Identical;
  }
  memcpy(sdk.actual_qd, original->actual_qd, sizeof(sdk.actual_qd));
  return Match::Repaired;
}
}  // namespace fairino_hardware
