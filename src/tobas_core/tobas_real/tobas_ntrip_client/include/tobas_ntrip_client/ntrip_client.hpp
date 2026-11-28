// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "tobas_ntrip_client/rtcm_scanner.hpp"

namespace tobas::ntrip
{
class NtripClient
{
public:
  explicit NtripClient() = default;

  NtripClient(NtripClient&& _other) = delete;
  NtripClient& operator=(NtripClient&& _other) = delete;
  NtripClient(const NtripClient& _other) = delete;
  NtripClient& operator=(const NtripClient& _other) = delete;

  ~NtripClient();

  /**
   * Connect to the NTRIP caster and start receiving data from the mount point.
   * This blocks the calling thread for a while because the communication is blocking.
   */
  bool initialize(
    const char* _server_ip,
    int _server_port,
    const char* _mount_point,
    const char* _user_name,
    const char* _password,
    double _latitude,
    double _longitude);

  /** Receive RTCM 3.x frames from the mount point without blocking. */
  std::vector<std::vector<uint8_t>> receiveRtcmData();

  /** Send data to the caster. */
  bool sendData(const std::string& _data);

  /** Send an NMEA GGA sentence to the caster. */
  bool sendNmeaGga(const std::string& _gga);

  bool isConnected() const;

  void closeSocket();

private:
  enum MountPointStatus : uint8_t
  {
    kSuccess,               ///< ICY 200 OK was received
    kGetSourceTable,        ///< A source table was received, so the mount point is unavailable
    kAuthorizationFailure,  ///< 401 was received
    kOtherFailure,
  };

  static constexpr int kTimeout = 5;  ///< Connection timeout [s]
  static constexpr size_t kChunkSize = 1024;
  static constexpr size_t kMountPointsToShow = 5;

  int socket_ = -1;
  bool is_connected_ = false;
  sockaddr_in server_address_ = {};

  char receive_buffer_[kChunkSize] = {};  ///< Buffer for received data, parsed after moving to the deque
  std::deque<uint8_t> receive_deque_;     ///< Received data, pushed at the back and popped from the front

  RtcmScanner scanner_;

  /** Create the HTTP request for the caster. */
  std::string
  createHttpRequest(const std::string& _mount_point, const std::string& _user_name, const std::string& _password);

  MountPointStatus
  checkMountPoint(const std::string& _mount_point, const std::string& _user_name, const std::string& _password);

  /** Connect over TCP with a timeout. */
  static bool connectWithTimeout(int _fd, const sockaddr* _addr, socklen_t _len, timeval& _timeout);

  /** Receive without blocking. Return the received size, or -1 on error or when no data was available. */
  static ssize_t nonblockReceive(int _fd, void* _buf, size_t _n, int _flags);

  static std::string base64Encode(const std::string& _src);
};
}  // namespace tobas::ntrip
