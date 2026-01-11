#include "tobas_ntrip_client/ntrip_client.hpp"

#include <arpa/inet.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <boost/beast/core/detail/base64.hpp>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

#include <tobas_ntrip_client/source_table_reader.hpp>

namespace ntrip
{
NtripClient::NtripClient()
{
}

bool NtripClient::initialize(
  const char* server_ip,
  const int& server_port,
  const char* mount_point,
  const char* user_name,
  const char* password,
  const double& latitude,
  const double& longitude)
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

  const auto mount_point_stat = checkMountPoint(mount_point, user_name, password);
  if (mount_point_stat == SUCCESS) {
    std::cout << "Successfully connected to the mount point." << std::endl;
    return true;
  }
  if (mount_point_stat == AUTHORIZATION_FAILURE || mount_point_stat == OTHER_FAILURE) {
    return false;
  }
  // 接続できていない場合でsource tableが返ってきている場合は，source tableから近くのmount pointを探して接続
  if (mount_point_stat == GET_SOURCE_TABLE) {
    std::cerr << "Receiving source table..." << std::endl;
    // read all source table data
    while (true) {
      auto receive_size = recv(socket_, receive_buffer_, kChunkSize, 0);
      if (receive_size <= 0) {
        break;
      }
      receive_deque_.insert(receive_deque_.end(), receive_buffer_, receive_buffer_ + receive_size);
    }

    // read source table data
    SourceTableReader source_table_reader(std::string(receive_deque_.begin(), receive_deque_.end()));

    // sort moint points by the distance from here
    auto sorted_moint_points = source_table_reader.sortMountPoints(latitude, longitude);

    // 距離が近い順にmount pointを表示 TODO : 自動でmount pointに接続する
    std::cout << "Recommended mount points : ";
    for (size_t i = 0; i < kMountPointsToShow; i++) {
      std::cout << sorted_moint_points[i] << " ";
    }
    std::cout << std::endl;
    
    // dequeから処理が終わったsource tableのデータを削除
    receive_deque_.resize(0);
    return false;
  }
  std::cerr << "Unexpected error." << std::endl;
  return false;
}

void NtripClient::receiveRtcmData()
{
  std::cout << "in receiveRtcm" << std::endl;
  while (true) {
    auto receive_size = nonblockReceive(socket_, receive_buffer_, kChunkSize, 0);
    if (receive_size <= 0) {
      break;
    }
    receive_deque_.insert(receive_deque_.end(), receive_buffer_, receive_buffer_ + receive_size);
  }
  std::cout << "deque size : " << receive_deque_.size() << std::endl;
  for (size_t i = 0; i < receive_deque_.size(); i++) {
    std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(receive_deque_[i]);
  }
  std::cout << std::dec << std::endl;
}

std::string
NtripClient::createHttpRequest(const std::string& mount_point, const std::string& user_name, const std::string& password)
{
  std::string basic_credentials = base64Encode(std::string(user_name) + ":" + std::string(password));
  std::string http_request =
    "GET /" + mount_point +
    " HTTP/1.0\r\n"
    "User-Agent: NTRIP str2str\r\n"  // str2strを名乗らないと何故かsource tableが返ってくるだけで通信に成功しない
    "Authorization: Basic " +
    basic_credentials +
    "\r\n"
    "\r\n";
  return http_request;
}

NtripClient::MountPointStatus
NtripClient::checkMountPoint(const std::string& mount_point, const std::string& user_name, const std::string& password)
{
  // NTRIP CasterへのHTTP requestを作成する
  const std::string http_request = createHttpRequest(mount_point, user_name, password);

  // HTTP requestを送信
  auto sended_size = send(socket_, http_request.c_str(), http_request.size(), 0);
  if (sended_size != static_cast<ssize_t>(http_request.size())) {
    {
      std::cerr << "Failed to send http request." << std::endl;
      return OTHER_FAILURE;
    }
  }

  // Get the response from the server
  auto receive_size = recv(socket_, receive_buffer_, kChunkSize, 0);
  if (receive_size <= 0) {
    std::cerr << "Received data size is strange." << std::endl;
    return OTHER_FAILURE;
  }

  receive_deque_.insert(receive_deque_.end(), receive_buffer_, receive_buffer_ + receive_size);
  // ICY 200 OKが含まれていれば接続できている
  if (std::string(receive_deque_.end() - receive_size, receive_deque_.end()).find("ICY 200 OK") != std::string::npos) {
    return SUCCESS;
  }

  // 接続できていない場合でsource tableが返ってきている場合
  if (std::string(receive_deque_.end() - receive_size, receive_deque_.end()).find("SOURCETABLE 200 OK") != std::string::npos) {
    std::cerr << "Received source table. Probably the mountpoint is not valid." << std::endl;
    return GET_SOURCE_TABLE;
  }

  if (std::string(receive_deque_.end() - receive_size, receive_deque_.end()).find("401") != std::string::npos) {
    std::cerr << "Received unauthorized response from the server. Check your user name and password." << std::endl;
    return AUTHORIZATION_FAILURE;
  }

  return OTHER_FAILURE;
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

ssize_t NtripClient::nonblockReceive(const int& fd, void* buf, size_t n, int flags)
{
  // 非同期に変更
  auto fcntl_flags = fcntl(fd, F_GETFL);
  if (fcntl_flags == -1) {
    std::cerr << "Failed in fcntl" << std::endl;
    return -1;
  }
  if (fcntl(fd, F_SETFL, fcntl_flags | O_NONBLOCK) == -1) {
    std::cerr << "Failed in fcntl" << std::endl;
    return -1;
  }

  // 受信処理
  auto result = recv(fd, buf, n, flags);

  // 同期に戻す
  if (fcntl(fd, F_SETFL, fcntl_flags) == -1) {
    std::cerr << "Failed in fcntl" << std::endl;
    return -1;
  }

  return result;
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
