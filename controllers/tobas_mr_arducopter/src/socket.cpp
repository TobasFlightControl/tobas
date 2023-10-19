#include <functional>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "../include/tobas_mr_arducopter/socket.hpp"

namespace tobas_mr_arducopter
{
ArduPilotSocket::ArduPilotSocket()
{
  // initialize socket udp socket
  fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  fcntl(fd_, F_SETFD, FD_CLOEXEC);
}

ArduPilotSocket::~ArduPilotSocket()
{
  if (fd_ != -1)
  {
    close(fd_);
    fd_ = -1;
  }
}

bool ArduPilotSocket::bind(const char* _address, const uint16_t _port)
{
  struct sockaddr_in sockaddr;
  makeSockAddr(_address, _port, sockaddr);

  if (::bind(fd_, (struct sockaddr*)&sockaddr, sizeof(sockaddr)) != 0)
  {
    shutdown(fd_, 0);
    close(fd_);
    return false;
  }
  int one = 1;
  setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));

  fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
  return true;
}

bool ArduPilotSocket::connect(const char* _address, const uint16_t _port)
{
  struct sockaddr_in sockaddr;
  makeSockAddr(_address, _port, sockaddr);

  if (::connect(fd_, (struct sockaddr*)&sockaddr, sizeof(sockaddr)) != 0)
  {
    shutdown(fd_, 0);
    close(fd_);
    return false;
  }
  int one = 1;
  setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));

  fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
  return true;
}

void ArduPilotSocket::makeSockAddr(
  const char* _address,
  const uint16_t _port,
  struct sockaddr_in& _sockaddr)
{
  memset(&_sockaddr, 0, sizeof(_sockaddr));

  _sockaddr.sin_port = htons(_port);
  _sockaddr.sin_family = AF_INET;
  _sockaddr.sin_addr.s_addr = inet_addr(_address);
}

ssize_t ArduPilotSocket::send(const void* _buf, size_t _size)
{
  return ::send(fd_, _buf, _size, 0);
}

ssize_t ArduPilotSocket::recv(void* _buf, const size_t _size, uint32_t _timeoutMs)
{
  fd_set fds;
  struct timeval tv;

  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  tv.tv_sec = _timeoutMs / 1000;
  tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

  if (select(fd_ + 1, &fds, NULL, NULL, &tv) != 1)
  {
    return -1;
  }

  return ::recv(fd_, _buf, _size, 0);
}
}  // namespace tobas_mr_arducopter
