#pragma once

#include <netinet/in.h>

#include <string>

namespace ntrip
{
class NtripClient
{
public:
  explicit NtripClient();
  bool initialize(const char* server_ip, const int& server_port);

private:
  static constexpr int kTimeout = 1; // 1 second
  int socket_;
  struct sockaddr_in server_address_;

  static bool connectWithTimeout(int fd, const sockaddr* addr, socklen_t len, timeval& timeout);
};
} // namespace ntrip
