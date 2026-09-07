// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_mission_items/mission.hpp"

#include <tobas_std_tools/byte.hpp>
#include <tobas_yaml_tools/convert/float64.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

#include "tobas_mission_items/mission_items.hpp"

/* Define a macro that assigns directly because packed struct elements cannot be bound to function arguments. */
#define LOAD_PACKED_FIELD(key, parent, field)                                                                          \
  (                                                                                                                    \
    [&]() noexcept -> bool                                                                                             \
    {                                                                                                                  \
      using FieldType = std::remove_cv_t<std::remove_reference_t<decltype(field)>>;                                    \
      const auto res = tobas::yaml::load<FieldType>((key), (parent));                                                  \
      if (!res) {                                                                                                      \
        std::cerr << res.error() << std::endl;                                                                         \
        return false;                                                                                                  \
      }                                                                                                                \
      (field) = *res;                                                                                                  \
      return true;                                                                                                     \
    }())

namespace tobas
{
namespace mission
{
namespace
{
// Mission item elements
constexpr char kTypeKey[] = "type";
constexpr char kDataKey[] = "data";

// Mission item types
constexpr char kTypeWaypoint[] = "waypoint";
constexpr char kTypeTakeoff[] = "takeoff";
constexpr char kTypeLand[] = "land";
constexpr char kTypeRtl[] = "rtl";

// Waypoint elements
constexpr char kWaypointLatitude[] = "latitude";
constexpr char kWaypointLongitude[] = "longitude";
constexpr char kWaypointAltitude[] = "altitude";
constexpr char kWaypointAltitudeFrame[] = "altitude_frame";
constexpr char kWaypointAutoHeading[] = "auto_heading";
constexpr char kWaypointStopAtWaypoint[] = "stop_at_waypoint";
constexpr char kWaypointMaxHorizontalVelocity[] = "max_horizontal_velocity";
constexpr char kWaypointMaxHorizontalAccel[] = "max_horizontal_accel";
constexpr char kWaypointMaxHorizontalJerk[] = "max_horizontal_jerk";
constexpr char kWaypointMaxVerticalVelocity[] = "max_vertical_velocity";
constexpr char kWaypointMaxVerticalAccel[] = "max_vertical_accel";
constexpr char kWaypointMaxVerticalJerk[] = "max_vertical_jerk";
constexpr char kWaypointMaxHeadingRate[] = "max_heading_rate";
constexpr char kWaypointMaxHeadingAccel[] = "max_heading_accel";
constexpr char kWaypointAcceptanceRadius[] = "acceptance_radius";
constexpr char kWaypointAltitudeTolerance[] = "altitude_tolerance";
constexpr char kWaypointTimeout[] = "timeout";

// Takeoff elements
constexpr char kTakeoffAltitude[] = "altitude";
constexpr char kTakeoffAltitudeFrame[] = "altitude_frame";
constexpr char kTakeoffMaxSpeed[] = "max_speed";
constexpr char kTakeoffMaxAccel[] = "max_accel";
constexpr char kTakeoffMaxJerk[] = "max_jerk";
constexpr char kTakeoffAltitudeTolerance[] = "altitude_tolerance";
constexpr char kTakeoffTimeout[] = "timeout";

// Land elements
constexpr char kLandSpeed[] = "speed";
constexpr char kLandTimeout[] = "timeout";

// RTL elements
constexpr char kRtlMinAltitude[] = "min_altitude";
constexpr char kRtlMaxHorizontalVelocity[] = "max_horizontal_velocity";
constexpr char kRtlMaxHorizontalAccel[] = "max_horizontal_accel";
constexpr char kRtlMaxHorizontalJerk[] = "max_horizontal_jerk";
constexpr char kRtlMaxVerticalVelocity[] = "max_vertical_velocity";
constexpr char kRtlMaxVerticalAccel[] = "max_vertical_accel";
constexpr char kRtlMaxVerticalJerk[] = "max_vertical_jerk";
constexpr char kRtlMaxHeadingRate[] = "max_heading_rate";
constexpr char kRtlMaxHeadingAccel[] = "max_heading_accel";
constexpr char kRtlAcceptanceRadius[] = "acceptance_radius";
constexpr char kRtlAltitudeTolerance[] = "altitude_tolerance";
constexpr char kRtlTimeout[] = "timeout";
}  // namespace

YAML::Node Mission::dump() const
{
  constexpr int kGnssPrecision = 12;

  YAML::Node mission_node(YAML::NodeType::Sequence);

  for (const auto& item : items) {
    YAML::Node item_node(YAML::NodeType::Map);
    YAML::Node data_node(YAML::NodeType::Map);

    switch (item.type) {
      case tobas::mission::Type::kWaypoint: {
        tobas::mission::Waypoint waypoint;
        if (!tobas::st::fromBytes(item.data, waypoint)) {
          std::cerr << "Failed to decode a waypoint mission." << std::endl;
          continue;
        }
        item_node[kTypeKey] = kTypeWaypoint;
        data_node[kWaypointLatitude] = tobas::yaml::format(waypoint.latitude, kGnssPrecision);
        data_node[kWaypointLongitude] = tobas::yaml::format(waypoint.longitude, kGnssPrecision);
        data_node[kWaypointAltitude] = tobas::yaml::format(waypoint.altitude);
        data_node[kWaypointAltitudeFrame] = waypoint.altitude_frame;
        data_node[kWaypointAutoHeading] = waypoint.auto_heading;
        data_node[kWaypointStopAtWaypoint] = waypoint.stop_at_waypoint;
        data_node[kWaypointMaxHorizontalVelocity] = tobas::yaml::format(waypoint.max_horizontal_velocity);
        data_node[kWaypointMaxHorizontalAccel] = tobas::yaml::format(waypoint.max_horizontal_accel);
        data_node[kWaypointMaxHorizontalJerk] = tobas::yaml::format(waypoint.max_horizontal_jerk);
        data_node[kWaypointMaxVerticalVelocity] = tobas::yaml::format(waypoint.max_vertical_velocity);
        data_node[kWaypointMaxVerticalAccel] = tobas::yaml::format(waypoint.max_vertical_accel);
        data_node[kWaypointMaxVerticalJerk] = tobas::yaml::format(waypoint.max_vertical_jerk);
        data_node[kWaypointMaxHeadingRate] = tobas::yaml::format(waypoint.max_heading_rate);
        data_node[kWaypointMaxHeadingAccel] = tobas::yaml::format(waypoint.max_heading_accel);
        data_node[kWaypointAcceptanceRadius] = tobas::yaml::format(waypoint.acceptance_radius);
        data_node[kWaypointAltitudeTolerance] = tobas::yaml::format(waypoint.altitude_tolerance);
        data_node[kWaypointTimeout] = tobas::yaml::format(waypoint.timeout);
        break;
      }
      case tobas::mission::Type::kTakeoff: {
        tobas::mission::Takeoff takeoff;
        if (!tobas::st::fromBytes(item.data, takeoff)) {
          std::cerr << "Failed to decode a takeoff mission." << std::endl;
          continue;
        }
        item_node[kTypeKey] = kTypeTakeoff;
        data_node[kTakeoffAltitude] = tobas::yaml::format(takeoff.altitude);
        data_node[kTakeoffAltitudeFrame] = takeoff.altitude_frame;
        data_node[kTakeoffMaxSpeed] = tobas::yaml::format(takeoff.max_speed);
        data_node[kTakeoffMaxAccel] = tobas::yaml::format(takeoff.max_accel);
        data_node[kTakeoffMaxJerk] = tobas::yaml::format(takeoff.max_jerk);
        data_node[kTakeoffAltitudeTolerance] = tobas::yaml::format(takeoff.altitude_tolerance);
        data_node[kTakeoffTimeout] = tobas::yaml::format(takeoff.timeout);
        break;
      }
      case tobas::mission::Type::kLand: {
        tobas::mission::Land land;
        if (!tobas::st::fromBytes(item.data, land)) {
          std::cerr << "Failed to decode a land mission." << std::endl;
          continue;
        }
        item_node[kTypeKey] = kTypeLand;
        data_node[kLandSpeed] = tobas::yaml::format(land.speed);
        data_node[kLandTimeout] = tobas::yaml::format(land.timeout);
        break;
      }
      case tobas::mission::Type::kReturnToLaunch: {
        tobas::mission::ReturnToLaunch rtl;
        if (!tobas::st::fromBytes(item.data, rtl)) {
          std::cerr << "Failed to decode a RTL mission." << std::endl;
          continue;
        }
        item_node[kTypeKey] = kTypeRtl;
        data_node[kRtlMinAltitude] = tobas::yaml::format(rtl.min_altitude);
        data_node[kRtlMaxHorizontalVelocity] = tobas::yaml::format(rtl.max_horizontal_velocity);
        data_node[kRtlMaxHorizontalAccel] = tobas::yaml::format(rtl.max_horizontal_accel);
        data_node[kRtlMaxHorizontalJerk] = tobas::yaml::format(rtl.max_horizontal_jerk);
        data_node[kRtlMaxVerticalVelocity] = tobas::yaml::format(rtl.max_vertical_velocity);
        data_node[kRtlMaxVerticalAccel] = tobas::yaml::format(rtl.max_vertical_accel);
        data_node[kRtlMaxVerticalJerk] = tobas::yaml::format(rtl.max_vertical_jerk);
        data_node[kRtlMaxHeadingRate] = tobas::yaml::format(rtl.max_heading_rate);
        data_node[kRtlMaxHeadingAccel] = tobas::yaml::format(rtl.max_heading_accel);
        data_node[kRtlAcceptanceRadius] = tobas::yaml::format(rtl.acceptance_radius);
        data_node[kRtlAltitudeTolerance] = tobas::yaml::format(rtl.altitude_tolerance);
        data_node[kRtlTimeout] = tobas::yaml::format(rtl.timeout);
        break;
      }
      default:
        throw;
    }

    item_node[kDataKey] = data_node;
    mission_node.push_back(item_node);
  }

  return mission_node;
}

bool Mission::load(const YAML::Node& mission_node)
{
  if (!mission_node.IsSequence()) {
    std::cerr << "YAML node type mismatch." << std::endl;
    return false;
  }

  for (const auto& item_node : mission_node) {
    const auto type = tobas::yaml::load<std::string>(kTypeKey, item_node);
    if (!type) {
      std::cerr << type.error() << std::endl;
      return false;
    }

    const auto data_node = item_node[kDataKey];
    if (!data_node.IsDefined()) {
      std::cerr << "\"" << kDataKey << "\" is not defined." << std::endl;
      return false;
    }

    tobas::mission::MissionItem item;

    if (*type == kTypeWaypoint) {
      tobas::mission::Waypoint waypoint;
      if (!LOAD_PACKED_FIELD(kWaypointLatitude, data_node, waypoint.latitude)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointLongitude, data_node, waypoint.longitude)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointAltitude, data_node, waypoint.altitude)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointAltitudeFrame, data_node, waypoint.altitude_frame)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointAutoHeading, data_node, waypoint.auto_heading)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointStopAtWaypoint, data_node, waypoint.stop_at_waypoint)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointMaxHorizontalVelocity, data_node, waypoint.max_horizontal_velocity)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointMaxHorizontalAccel, data_node, waypoint.max_horizontal_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointMaxHorizontalJerk, data_node, waypoint.max_horizontal_jerk)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointMaxVerticalVelocity, data_node, waypoint.max_vertical_velocity)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointMaxVerticalAccel, data_node, waypoint.max_vertical_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointMaxVerticalJerk, data_node, waypoint.max_vertical_jerk)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointMaxHeadingRate, data_node, waypoint.max_heading_rate)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointMaxHeadingAccel, data_node, waypoint.max_heading_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointAcceptanceRadius, data_node, waypoint.acceptance_radius)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointAltitudeTolerance, data_node, waypoint.altitude_tolerance)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kWaypointTimeout, data_node, waypoint.timeout)) {
        return false;
      }
      item.type = tobas::mission::Type::kWaypoint;
      item.data = tobas::st::toBytes(waypoint);
    }
    else if (*type == kTypeTakeoff) {
      tobas::mission::Takeoff takeoff;
      if (!LOAD_PACKED_FIELD(kTakeoffAltitude, data_node, takeoff.altitude)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kTakeoffAltitudeFrame, data_node, takeoff.altitude_frame)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kTakeoffMaxSpeed, data_node, takeoff.max_speed)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kTakeoffMaxAccel, data_node, takeoff.max_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kTakeoffMaxJerk, data_node, takeoff.max_jerk)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kTakeoffAltitudeTolerance, data_node, takeoff.altitude_tolerance)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kTakeoffTimeout, data_node, takeoff.timeout)) {
        return false;
      }
      item.type = tobas::mission::Type::kTakeoff;
      item.data = tobas::st::toBytes(takeoff);
    }
    else if (*type == kTypeLand) {
      tobas::mission::Land land;
      if (!LOAD_PACKED_FIELD(kLandSpeed, data_node, land.speed)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kLandTimeout, data_node, land.timeout)) {
        return false;
      }
      item.type = tobas::mission::Type::kLand;
      item.data = tobas::st::toBytes(land);
    }
    else if (*type == kTypeRtl) {
      tobas::mission::ReturnToLaunch rtl;
      if (!LOAD_PACKED_FIELD(kRtlMinAltitude, data_node, rtl.min_altitude)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kRtlMaxHorizontalVelocity, data_node, rtl.max_horizontal_velocity)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kRtlMaxHorizontalAccel, data_node, rtl.max_horizontal_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kRtlMaxHorizontalJerk, data_node, rtl.max_horizontal_jerk)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kRtlMaxVerticalVelocity, data_node, rtl.max_vertical_velocity)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kRtlMaxVerticalAccel, data_node, rtl.max_vertical_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kRtlMaxVerticalJerk, data_node, rtl.max_vertical_jerk)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kRtlMaxHeadingRate, data_node, rtl.max_heading_rate)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kRtlMaxHeadingAccel, data_node, rtl.max_heading_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kRtlAcceptanceRadius, data_node, rtl.acceptance_radius)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kRtlAltitudeTolerance, data_node, rtl.altitude_tolerance)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(kRtlTimeout, data_node, rtl.timeout)) {
        return false;
      }
      item.type = tobas::mission::Type::kReturnToLaunch;
      item.data = tobas::st::toBytes(rtl);
    }
    else {
      std::cerr << "Invalid mission item type: " << *type << std::endl;
      return false;
    }

    items.push_back(item);
  }

  return true;
}
}  // namespace mission
}  // namespace tobas
