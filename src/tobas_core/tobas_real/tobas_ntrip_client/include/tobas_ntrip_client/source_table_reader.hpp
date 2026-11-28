// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace tobas::ntrip
{
/** Ref: [Example source table](http://rtk2go.com:2101/) */
class SourceTableReader
{
public:
  /** Parse the source table given as a string. */
  explicit SourceTableReader(const std::string& _data);

  /** Sort the mount points by their distance from the given position in degrees, nearest first. */
  std::vector<std::string> sortMountPoints(double _latitude, double _longitude);

private:
  enum Index : uint8_t
  {
    kType = 0,        ///< e.g. STR
    kMountPoint,      ///< Mount point name, e.g. NEAR-JPNc
    kIdentifier,      ///< Source ID or region name, e.g. Japan Central
    kFormat,          ///< Data format, e.g. RTCM 3.2
    kFormatDetails,   ///< Format details, e.g. 1002(1),1006(10)
    kCarrier,         ///< Carrier phase: 0 = none, 1 = L1, 2 = L1 and L2
    kNavSystem,       ///< GNSS names
    kNetwork,         ///< Network name
    kCountry,         ///< ISO 3166 country code, e.g. JPN
    kLatitude,        ///< North latitude [deg]
    kLongitude,       ///< East longitude [deg]
    kNmea,            ///< Client may send its position as NMEA: 0 = no, 1 = yes
    kSolution,        ///< Reference station: 0 = single, 1 = network
    kGenerator,       ///< Generating software, e.g. sNTRIP
    kComprEncryp,     ///< Compression and encryption algorithm
    kAuthentication,  ///< Authentication: N = none, B = basic, D = digest
    kFee,             ///< Fee: N = no, Y = yes
    kBitrate,         ///< Bit rate [bps]
    kMisc,            ///< Miscellaneous
  };

  std::vector<std::vector<std::string>> mount_points_;  ///< Records read from the source table
};
}  // namespace tobas::ntrip
