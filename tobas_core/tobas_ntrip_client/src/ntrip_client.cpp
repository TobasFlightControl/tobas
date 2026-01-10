#include "tobas_ntrip_client/ntrip_client.hpp"

#include <arpa/inet.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <boost/beast/core/detail/base64.hpp>
#include <iostream>
#include <string>

namespace ntrip
{
NtripClient::NtripClient()
{
}

bool NtripClient::initialize(const char* server_ip, const int& server_port, const char* mount_point, const char* user_name, const char* password)
{
  // TCP通信用のsocketを作成
  socket_ = socket(AF_INET, SOCK_STREAM, 0);

  // NTRIP Casterのaddressを設定
  server_address_.sin_addr.s_addr = inet_addr(server_ip);
  server_address_.sin_port = htons(server_port);
  server_address_.sin_family = AF_INET;

  // NTRIP Casterに接続する
  timeval timeout;
  timeout.tv_sec = kTimeout;
  timeout.tv_usec = 0;
  if (!connectWithTimeout(socket_, reinterpret_cast<sockaddr*>(&server_address_), sizeof(server_address_), timeout)) {
    std::cerr << "Failed in connectWithTimeout" << std::endl;
    return false;
  }

  // NTRIP CasterへのHTTP requestを作成する
  std::string basic_credentials = base64Encode(std::string(user_name) + ":" + std::string(password));
  std::string http_request = "GET /" + std::string(mount_point) + " HTTP/1.0\r\nUser-Agent: NTRIP ntrip_client_ros\r\n" + "Authorization: Basic " + basic_credentials + "\r\n";

  // HTTP requestを送信
  send(socket_, http_request.c_str(), http_request.size(), 0);

  // Get the response from the server
  char buf[kChunkSize];
  auto receive_size = recv(socket_, buf, kChunkSize, 0);
  for (ssize_t i = 0; i < receive_size; i++) { // debug用, 消す予定
    std::cout << buf[i];
  }
  std::cout << std::endl;
  if (std::string(buf).find("SOURCETABLE 200 OK") != std::string::npos) {
    std::cerr << "Received source table. Probably the mountpoint is not valid." << std::endl;
    return false;
  } else if (std::string(buf).find("401") != std::string::npos) {
    std::cerr << "Received unauthorized response from the server. Check your user name and password." << std::endl;
    return false;
  }
  return true;
}

bool NtripClient::connectWithTimeout(int fd, const sockaddr* addr, socklen_t len, timeval& timeout)
{
  // 接続前に一度非同期に変更
  auto flags = fcntl(fd, F_GETFL);
  if (flags == -1) {
    std::cerr << "Failed in fcntl" << std::endl;
    return false;
  }
  if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
    std::cerr << "Failed in fcntl" << std::endl;
    return false;
  }

  // 接続
  auto result = connect(fd, addr, len);
  if (result == EWOULDBLOCK) {
    if (EINPROGRESS == errno) {
      // 非同期接続成功だとここに入る。select()で完了を待つ。
      errno = 0;
    }
    else {
      // 接続失敗 同期に戻す。
      fcntl(fd, F_SETFL, flags);
      std::cerr << "Failed in fcntl" << std::endl;
      return false;
    }
  }

  // 同期に戻す。
  if (fcntl(fd, F_SETFL, flags) == -1) {
    std::cerr << "Failed in fcntl" << std::endl;
    return false;
  }

  // セレクトで待つ
  fd_set read_fd, write_fd, err_fd;
  FD_ZERO(&read_fd);
  FD_ZERO(&write_fd);
  FD_ZERO(&err_fd);
  FD_SET(fd, &read_fd);
  FD_SET(fd, &write_fd);
  FD_SET(fd, &err_fd);
  int sock_num = select(fd + 1, &read_fd, &write_fd, &err_fd, &timeout);
  if (sock_num == 0) {
    // timeout error
    std::cerr << "Connection timeout" << std::endl;
    return false;
  }
  else if (FD_ISSET(fd, &read_fd) || FD_ISSET(fd, &write_fd)) {
    // 読み書きできる状態
  }
  else {
    std::cerr << "Connection error" << std::endl;
    return false;
  }

  // ソケットエラー確認 SO_ERRORを読み出して0か確認
  int optval = 0;
  socklen_t optlen = (socklen_t)sizeof(optval);
  errno = 0;
  if (getsockopt(fd, SOL_SOCKET, SO_ERROR, (void*)&optval, &optlen) < 0) {
    std::cerr << "Failed in getsockopt" << std::endl;
    return false;
  }
  if (optval != 0) {
    std::cerr << "Socket Error" << std::endl;
    return false;
  }

  return true;
}

std::string NtripClient::base64Encode(const std::string& src)
{
  std::string dst;
  dst.resize(boost::beast::detail::base64::encoded_size(src.size()));
  auto real_size = boost::beast::detail::base64::encode(&dst[0], src.c_str(), src.size());
  dst.resize(real_size);
  return dst;
}
}  // namespace ntrip
