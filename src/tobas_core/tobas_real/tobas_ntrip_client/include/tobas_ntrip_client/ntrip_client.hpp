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
/** A client of an NTRIP caster that streams RTCM 3.x corrections from a mount point. */
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
   *
   * @param _server_ip Host name or dotted-decimal address of the caster
   * @param _server_port TCP port of the caster
   * @param _mount_point Mount point to request
   * @param _user_name User name for basic authentication, or an empty string or `none` to skip it
   * @param _password Password for basic authentication
   * @param _latitude Geodetic latitude used to recommend mount points when the requested one is unavailable [deg]
   * @param _longitude Geodetic longitude used for the same purpose [deg]
   * @return false if the connection could not be established.
   *
   * @note This blocks the calling thread while it waits for the response of the caster.
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

  /** Send an NMEA GGA sentence to the caster, appending `\r\n` when it is missing. */
  bool sendNmeaGga(const std::string& _gga);

  bool isConnected() const;

  void closeSocket();

private:
  /** Result of requesting a mount point from the caster. */
  enum MountPointStatus : uint8_t
  {
    kSuccess,               ///< `ICY 200 OK` was received, so the mount point is streaming.
    kGetSourceTable,        ///< A source table was received, so the caster is reachable but the mount point is not.
    kAuthorizationFailure,  ///< `401` was received, so the user name or the password is wrong.
    kOtherFailure,
  };

  static constexpr int kTimeout = 5;  ///< Connection timeout [s]
  static constexpr size_t kChunkSize = 1024;
  static constexpr size_t kMountPointsToShow = 5;

  int socket_ = -1;
  bool is_connected_ = false;
  sockaddr_in server_address_ = {};

  /** Buffer that `recv()` writes into. The data is parsed after it is moved to `receive_deque_`. */
  char receive_buffer_[kChunkSize] = {};

  /** Received data, pushed at the back and popped from the front once it has been parsed. */
  std::deque<uint8_t> receive_deque_;

  RtcmScanner scanner_;

  /** Build the HTTP request that asks the caster for a mount point. */
  std::string
  createHttpRequest(const std::string& _mount_point, const std::string& _user_name, const std::string& _password);

  /** Request a mount point and classify the response of the caster. */
  MountPointStatus
  checkMountPoint(const std::string& _mount_point, const std::string& _user_name, const std::string& _password);

  /** Connect over TCP, giving up once `_timeout` has elapsed. */
  static bool connectWithTimeout(int _fd, const sockaddr* _addr, socklen_t _len, timeval& _timeout);

  /** Receive without blocking. Return the received size, or -1 on error or when no data was available. */
  static ssize_t nonblockReceive(int _fd, void* _buf, size_t _n, int _flags);

  static std::string base64Encode(const std::string& _src);
};
}  // namespace tobas::ntrip
