#include <cstdio>
#include <cstdlib>
#include <sys/socket.h>

// SDK tests interpose SendFrame. Fail before any accidental networking can occur.
static void unexpected_network()
{
  std::fputs("Unexpected network call in offline SDK test\n", stderr);
  std::abort();
}

extern "C" int connect(int, const sockaddr *, socklen_t) {unexpected_network(); return -1;}
extern "C" ssize_t send(int, const void *, size_t, int) {unexpected_network(); return -1;}
extern "C" ssize_t sendto(int, const void *, size_t, int, const sockaddr *, socklen_t)
{
  unexpected_network();
  return -1;
}
