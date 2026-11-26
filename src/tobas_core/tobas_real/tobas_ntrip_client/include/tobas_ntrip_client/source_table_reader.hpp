// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace tobas::ntrip
{
/**
 * A reader of the source table that an NTRIP caster returns when the requested mount point is not available.
 *
 * Ref: [Example source table](http://rtk2go.com:2101/)
 */
class SourceTableReader
{
public:
  /** Parse the source table given as a string. */
  explicit SourceTableReader(const std::string& _data);

  /**
   * Sort the mount points by their distance from the given position.
   *
   * @param _latitude Geodetic latitude [deg]
   * @param _longitude Geodetic longitude [deg]
   * @return Mount point names, nearest first.
   */
  std::vector<std::string> sortMountPoints(double _latitude, double _longitude);

private:
  /** Column indices of an `STR` record in a source table. */
  enum Index : uint8_t
  {
    kType = 0,        ///< Record type, e.g. `STR`
    kMountPoint,      ///< Mount point name, e.g. `NEAR-JPNc`
    kIdentifier,      ///< Source identifier such as a region name, e.g. `Japan Central`
    kFormat,          ///< Data format, e.g. `RTCM 3.2`
    kFormatDetails,   ///< Format details, e.g. `1002(1),1006(10)`
    kCarrier,         ///< Carrier phase information: 0 = none, 1 = L1, 2 = L1 and L2
    kNavSystem,       ///< GNSS names
    kNetwork,         ///< Network name
    kCountry,         ///< ISO 3166 country code, e.g. `JPN`
    kLatitude,        ///< Latitude north [deg]
    kLongitude,       ///< Longitude east [deg]
    kNmea,            ///< Whether the client may send its own position as NMEA: 0 = no, 1 = yes
    kSolution,        ///< Reference station type: 0 = single, 1 = network
    kGenerator,       ///< Software that generates the stream, e.g. `sNTRIP`
    kComprEncryp,     ///< Compression and encryption algorithm
    kAuthentication,  ///< Authentication: N = none, B = basic, D = digest
    kFee,             ///< Whether the stream is charged for: N = no, Y = yes
    kBitrate,         ///< Bit rate [bps]
    kMisc,            ///< Miscellaneous
  };

  std::vector<std::vector<std::string>> mount_points_;  ///< Records read from the source table
};
}  // namespace tobas::ntrip
