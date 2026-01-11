#include "tobas_ntrip_client/ntrip_client.hpp"

#include <arpa/inet.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <boost/beast/core/detail/base64.hpp>
#include <iostream>
#include <string>
#include <vector>

#include <tobas_ntrip_client/source_table_reader.hpp>

namespace ntrip
{
NtripClient::NtripClient()
{
}

bool NtripClient::initialize(const char* server_ip, const int& server_port, const char* mount_point, const char* user_name, const char* password, const double& latitude, const double& longitude)
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
  const std::string http_request = createHttpRequest(mount_point, user_name, password);

  // HTTP requestを送信
  auto sended_size = send(socket_, http_request.c_str(), http_request.size(), 0);
  if (sended_size != static_cast<ssize_t>(http_request.size())) {{
    std::cerr << "Failed to send http request." << std::endl;
    return false;
  }}

  // Get the response from the server
  char buf[kChunkSize];
  auto receive_size = recv(socket_, buf, kChunkSize, 0);

  // ICY 200 OKが含まれていれば接続できている
  if (std::string(buf).find("ICY 200 OK") != std::string::npos) {
    return true;
  }

  // 接続できていない場合でsource tableが返ってきている場合は，source tableから近くのmount pointを探して接続
  if (std::string(buf).find("SOURCETABLE 200 OK") != std::string::npos) {
    std::cerr << "Received source table. Probably the mountpoint is not valid. Receiving source table..." << std::endl;
    // read all source table data
    std::vector<char> source_table;
    source_table.insert(source_table.end(), buf, buf + receive_size);
    while (receive_size > 0) {
      receive_size = recv(socket_, buf, kChunkSize, 0);
      source_table.insert(source_table.end(), buf, buf + receive_size);
    }

    // read source table data
    SourceTableReader source_table_reader(std::string(source_table.begin(), source_table.end()));

    // sort moint points by the distance from here
    auto sorted_moint_points = source_table_reader.sortMountPoints(latitude, longitude);

    // 距離が近い順にmount pointを試していく
    for (size_t i = 0; i < 3; i++) {
      std::string mount_point_to_test = sorted_moint_points[i];
      std::cout << "Trying " << mount_point_to_test << ", i = " << i << std::endl;

      // 先程と同じようにしてNTRIP CasterへHTTP requestを送信してresponseをチェック
      const std::string http_request_again = createHttpRequest(mount_point_to_test, user_name, password);
      send(socket_, http_request_again.c_str(), http_request_again.size(), 0);
      receive_size = recv(socket_, buf, kChunkSize, 0);
      // ICY 200 OKが含まれていれば接続できている
      if (std::string(buf).find("ICY 200 OK") != std::string::npos) {
        return true;
      }
      if (std::string(buf).find("SOURCETABLE 200 OK") != std::string::npos) {
        std::cerr << "Received source table. Probably the mountpoint is not valid." << std::endl;
      }
    }
    return false;
  } else if (std::string(buf).find("401") != std::string::npos) {
    std::cerr << "Received unauthorized response from the server. Check your user name and password." << std::endl;
    return false;
  }
  return true;
}

std::string NtripClient::createHttpRequest(const std::string& mount_point, const std::string& user_name, const std::string& password)
{
  std::string basic_credentials = base64Encode(std::string(user_name) + ":" + std::string(password));
  std::string http_request =
    "GET /" + mount_point + " HTTP/1.0\r\n"
    "User-Agent: NTRIP str2str\r\n" // str2strを名乗らないと何故かsource tableが返ってくるだけで通信に成功しない
    "Authorization: Basic " + basic_credentials + "\r\n"
    "\r\n";
  return http_request;
}

bool NtripClient::connectWithTimeout(const int& fd, const sockaddr* addr, socklen_t len, timeval& timeout)
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
    if (errno == EINPROGRESS) {
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
