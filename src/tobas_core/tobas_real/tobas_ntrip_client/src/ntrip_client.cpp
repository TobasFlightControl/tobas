// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ntrip_client/ntrip_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/select.h>
#include <unistd.h>

#include <cerrno>
#include <iostream>

#include <boost/beast/core/detail/base64.hpp>

#include "tobas_ntrip_client/source_table_reader.hpp"

namespace tobas::ntrip
{
NtripClient::~NtripClient()
{
  closeSocket();
}

bool NtripClient::initialize(
  const char* _server_ip,
  int _server_port,
  const char* _mount_point,
  const char* _user_name,
  const char* _password,
  double _latitude,
  double _longitude)
{
  closeSocket();

  // Create the socket for the TCP connection.
  socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_ < 0) {
    std::cerr << "Failed to create socket." << std::endl;
    return false;
  }

  // Resolve the address of the caster, accepting both a dotted-decimal address and a host name.
  server_address_.sin_family = AF_INET;
  server_address_.sin_port = htons(_server_port);
  const auto addr = inet_addr(_server_ip);
  if (addr != INADDR_NONE) {
    server_address_.sin_addr.s_addr = addr;
  }
  else {
    hostent* he = gethostbyname(_server_ip);
    if (he == nullptr || he->h_addr_list[0] == nullptr) {
      std::cerr << "Failed to resolve hostname: " << _server_ip << std::endl;
      closeSocket();
      return false;
    }
    server_address_.sin_addr = *reinterpret_cast<in_addr*>(he->h_addr_list[0]);
  }

  // Connect to the caster.
  timeval timeout;
  timeout.tv_sec = kTimeout;
  timeout.tv_usec = 0;
  if (!connectWithTimeout(socket_, reinterpret_cast<sockaddr*>(&server_address_), sizeof(server_address_), timeout)) {
    std::cerr << "Failed in connectWithTimeout" << std::endl;
    closeSocket();
    return false;
  }

  const auto mount_point_status = checkMountPoint(_mount_point, _user_name, _password);
  if (mount_point_status == kSuccess) {
    std::cout << "Successfully connected to the mount point." << std::endl;
    is_connected_ = true;
    return true;
  }
  if (mount_point_status == kAuthorizationFailure || mount_point_status == kOtherFailure) {
    closeSocket();
    return false;
  }

  // The mount point is unavailable but the caster returned its source table,
  // so recommend the mount points nearest to the given position.
  if (mount_point_status == kGetSourceTable) {
    std::cerr << "Receiving source table..." << std::endl;
    while (true) {
      const auto receive_size = recv(socket_, receive_buffer_, kChunkSize, 0);
      if (receive_size <= 0) {
        break;
      }
      receive_deque_.insert(receive_deque_.end(), receive_buffer_, receive_buffer_ + receive_size);
    }

    SourceTableReader source_table_reader(std::string(receive_deque_.begin(), receive_deque_.end()));
    const auto sorted_mount_points = source_table_reader.sortMountPoints(_latitude, _longitude);

    // TODO: Connect to the nearest mount point automatically.
    std::cout << "Recommended mount points : ";
    for (size_t i = 0; i < kMountPointsToShow && i < sorted_mount_points.size(); i++) {
      std::cout << sorted_mount_points[i] << " ";
    }
    std::cout << std::endl;

    closeSocket();
    return false;
  }

  std::cerr << "Unexpected error." << std::endl;
  closeSocket();
  return false;
}

std::vector<std::vector<uint8_t>> NtripClient::receiveRtcmData()
{
  if (!is_connected_ || socket_ < 0) {
    return {};
  }

  // Drain everything the mount point has sent so far.
  while (true) {
    errno = 0;
    const auto receive_size = nonblockReceive(socket_, receive_buffer_, kChunkSize, 0);
    if (receive_size == 0) {
      std::cerr << "NTRIP caster closed connection." << std::endl;
      closeSocket();
      break;
    }
    if (receive_size < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      std::cerr << "Socket error on receive: " << errno << std::endl;
      closeSocket();
      break;
    }
    receive_deque_.insert(receive_deque_.end(), receive_buffer_, receive_buffer_ + receive_size);
  }

  // Split the received bytes into RTCM 3.x frames, dropping the frames whose checksum does not match.
  std::vector<std::vector<uint8_t>> packets;
  while (!receive_deque_.empty()) {
    scanner_.update(receive_deque_.front());
    receive_deque_.pop_front();
    if (scanner_.state() == RtcmScanner::kDone) {
      if (scanner_.checkSum()) {
        packets.emplace_back(scanner_.getPreamble(), scanner_.getEnd());
      }
      scanner_.reset();
    }
  }

  return packets;
}

bool NtripClient::sendData(const std::string& _data)
{
  if (!is_connected_ || socket_ < 0) {
    return false;
  }
  const auto sent_size = send(socket_, _data.c_str(), _data.size(), MSG_NOSIGNAL);
  if (sent_size < 0) {
    std::cerr << "Failed to send data to NTRIP caster." << std::endl;
    closeSocket();
    return false;
  }
  return sent_size == static_cast<ssize_t>(_data.size());
}

bool NtripClient::sendNmeaGga(const std::string& _gga)
{
  std::string formatted = _gga;
  if (formatted.size() < 2 || formatted.substr(formatted.size() - 2) != "\r\n") {
    if (!formatted.empty() && formatted.back() == '\n') {
      formatted.pop_back();
    }
    formatted += "\r\n";
  }
  return sendData(formatted);
}

bool NtripClient::isConnected() const
{
  return is_connected_;
}

void NtripClient::closeSocket()
{
  is_connected_ = false;
  if (socket_ >= 0) {
    close(socket_);
    socket_ = -1;
  }
  receive_deque_.clear();
  scanner_.reset();
}

std::string NtripClient::createHttpRequest(
  const std::string& _mount_point,
  const std::string& _user_name,
  const std::string& _password)
{
  std::string http_request = "GET /" + _mount_point + " HTTP/1.0\r\n";
  http_request += "User-Agent: NTRIP str2str\r\n";
  http_request += "Accept: */*\r\n";
  http_request += "Connection: close\r\n";
  if (!_user_name.empty() && _user_name != "none") {
    const std::string basic_credentials = base64Encode(_user_name + ":" + _password);
    http_request += "Authorization: Basic " + basic_credentials + "\r\n";
  }
  http_request += "\r\n";
  return http_request;
}

NtripClient::MountPointStatus
NtripClient::checkMountPoint(const std::string& _mount_point, const std::string& _user_name, const std::string& _password)
{
  const std::string http_request = createHttpRequest(_mount_point, _user_name, _password);

  const auto sent_size = send(socket_, http_request.c_str(), http_request.size(), 0);
  if (sent_size != static_cast<ssize_t>(http_request.size())) {
    std::cerr << "Failed to send http request." << std::endl;
    return kOtherFailure;
  }

  // Get the response from the caster.
  const auto receive_size = recv(socket_, receive_buffer_, kChunkSize, 0);
  if (receive_size <= 0) {
    std::cerr << "Received data size is strange." << std::endl;
    return kOtherFailure;
  }

  receive_deque_.insert(receive_deque_.end(), receive_buffer_, receive_buffer_ + receive_size);

  const std::string response_str(receive_deque_.end() - receive_size, receive_deque_.end());

  // The caster is reachable but it returned its source table instead of the requested stream.
  if (response_str.find("SOURCETABLE 200 OK") != std::string::npos) {
    std::cerr << "Received source table. Probably the mountpoint is not valid." << std::endl;
    return kGetSourceTable;
  }

  if (response_str.find("401") != std::string::npos) {
    std::cerr << "Received unauthorized response from the server. Check your user name and password." << std::endl;
    return kAuthorizationFailure;
  }

  // `ICY 200 OK`, `HTTP/1.1 200 OK` and `HTTP/1.0 200 OK` all mean that the stream has started.
  if (
    response_str.find("ICY 200 OK") != std::string::npos || response_str.find("HTTP/1.1 200 OK") != std::string::npos ||
    response_str.find("HTTP/1.0 200 OK") != std::string::npos || response_str.find("200 OK") != std::string::npos) {
    // Drop the response header so that only the stream itself is parsed as RTCM.
    const auto header_end = response_str.find("\r\n\r\n");
    if (header_end != std::string::npos) {
      size_t pop_count = header_end + 4;
      if (pop_count > receive_deque_.size()) {
        pop_count = receive_deque_.size();
      }
      receive_deque_.erase(receive_deque_.begin(), receive_deque_.begin() + pop_count);
    }
    else {
      receive_deque_.clear();
    }
    return kSuccess;
  }

  return kOtherFailure;
}

bool NtripClient::connectWithTimeout(int _fd, const sockaddr* _addr, socklen_t _len, timeval& _timeout)
{
  // Make the socket non-blocking so that `connect()` returns immediately.
  const auto flags = fcntl(_fd, F_GETFL);
  if (flags == -1) {
    std::cerr << "Failed in fcntl" << std::endl;
    return false;
  }
  if (fcntl(_fd, F_SETFL, flags | O_NONBLOCK) == -1) {
    std::cerr << "Failed in fcntl" << std::endl;
    return false;
  }

  // `connect()` returns 0 on immediate success and -1 on failure, so the status must be read from `errno`.
  // `EINPROGRESS` is not a failure here: the handshake has started and `select()` below waits for it to complete.
  errno = 0;
  if (connect(_fd, _addr, _len) == -1 && errno != EINPROGRESS) {
    // The connection failed, so restore the blocking mode.
    std::cerr << "Failed in connect" << std::endl;
    fcntl(_fd, F_SETFL, flags);
    return false;
  }
  errno = 0;

  // Restore the blocking mode.
  if (fcntl(_fd, F_SETFL, flags) == -1) {
    std::cerr << "Failed in fcntl" << std::endl;
    return false;
  }

  // Wait for the socket to become ready.
  fd_set read_fd;
  fd_set write_fd;
  fd_set err_fd;
  FD_ZERO(&read_fd);
  FD_ZERO(&write_fd);
  FD_ZERO(&err_fd);
  FD_SET(_fd, &read_fd);
  FD_SET(_fd, &write_fd);
  FD_SET(_fd, &err_fd);
  const int sock_num = select(_fd + 1, &read_fd, &write_fd, &err_fd, &_timeout);
  if (sock_num < 0) {
    std::cerr << "Failed in select" << std::endl;
    return false;
  }
  if (sock_num == 0) {
    std::cerr << "Connection timeout" << std::endl;
    return false;
  }
  if (!FD_ISSET(_fd, &read_fd) && !FD_ISSET(_fd, &write_fd)) {
    std::cerr << "Connection error" << std::endl;
    return false;
  }

  // Make sure that `SO_ERROR` is clear.
  int optval = 0;
  socklen_t optlen = static_cast<socklen_t>(sizeof(optval));
  errno = 0;
  if (getsockopt(_fd, SOL_SOCKET, SO_ERROR, &optval, &optlen) < 0) {
    std::cerr << "Failed in getsockopt" << std::endl;
    return false;
  }
  if (optval != 0) {
    std::cerr << "Socket Error" << std::endl;
    return false;
  }

  return true;
}

ssize_t NtripClient::nonblockReceive(int _fd, void* _buf, size_t _n, int _flags)
{
  const auto fcntl_flags = fcntl(_fd, F_GETFL);
  if (fcntl_flags == -1) {
    std::cerr << "Failed in fcntl" << std::endl;
    return -1;
  }
  if (fcntl(_fd, F_SETFL, fcntl_flags | O_NONBLOCK) == -1) {
    std::cerr << "Failed in fcntl" << std::endl;
    return -1;
  }

  const auto result = recv(_fd, _buf, _n, _flags);

  if (fcntl(_fd, F_SETFL, fcntl_flags) == -1) {
    std::cerr << "Failed in fcntl" << std::endl;
    return -1;
  }

  return result;
}

std::string NtripClient::base64Encode(const std::string& _src)
{
  std::string dst;
  dst.resize(boost::beast::detail::base64::encoded_size(_src.size()));
  const auto real_size = boost::beast::detail::base64::encode(dst.data(), _src.c_str(), _src.size());
  dst.resize(real_size);
  return dst;
}
}  // namespace tobas::ntrip
