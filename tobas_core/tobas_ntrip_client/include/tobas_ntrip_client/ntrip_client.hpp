#pragma once

#include <netinet/in.h>

#include <string>

namespace ntrip
{
class NtripClient
{
public:
  explicit NtripClient();
  bool initialize(const char* server_ip, const int& server_port, const char* mount_point, const char* user_name, const char* password);

private:
  static constexpr int kTimeout = 1; // 1 second
  static constexpr size_t kChunkSize = 1024;
  int socket_;
  struct sockaddr_in server_address_;

  static bool connectWithTimeout(int fd, const sockaddr* addr, socklen_t len, timeval& timeout);
  static std::string base64Encode(const std::string& src);
};
} // namespace ntrip
