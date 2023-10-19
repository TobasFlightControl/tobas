#pragma once

#include <cinttypes>
#include <sys/socket.h>

namespace tobas_mr_arducopter
{
class ArduPilotSocket
{
public:
  explicit ArduPilotSocket();
  ~ArduPilotSocket();

  /// \brief Bind to an adress and port
  /// \param[in] _address Address to bind to.
  /// \param[in] _port Port to bind to.
  /// \return True on success.
  bool bind(const char* _address, const uint16_t _port);

  /// \brief Connect to an adress and port
  /// \param[in] _address Address to connect to.
  /// \param[in] _port Port to connect to.
  /// \return True on success.
  bool connect(const char* _address, const uint16_t _port);

  /// \brief Make a socket
  /// \param[in] _address Socket address.
  /// \param[in] _port Socket port
  /// \param[out] _sockaddr New socket address structure.
  void makeSockAddr(const char* _address, const uint16_t _port, struct sockaddr_in& _sockaddr);

  ssize_t send(const void* _buf, size_t _size);

  /// \brief Receive data
  /// \param[out] _buf Buffer that receives the data.
  /// \param[in] _size Size of the buffer.
  /// \param[in] _timeoutMS Milliseconds to wait for data.
  ssize_t recv(void* _buf, const size_t _size, uint32_t _timeoutMs);

private:
  int fd_;
};
}  // namespace tobas_mr_arducopter
