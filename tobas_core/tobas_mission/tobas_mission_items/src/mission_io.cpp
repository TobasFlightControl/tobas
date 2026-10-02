// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_mission_items/mission_io.hpp"

#include <tobas_std_tools/byte.hpp>
#include <tobas_yaml_tools/convert/float64.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

#include "tobas_mission_items/mission_items.hpp"

/** Packed fields cannot be bound to references; assign only after conversion succeeds. */
#define LOAD_PACKED_FIELD(_key, _parent, _field)                                                                       \
  do {                                                                                                                 \
    using FieldType = std::remove_cv_t<std::remove_reference_t<decltype(_field)>>;                                     \
    const auto field = tobas::yaml::load<FieldType>((_key), (_parent));                                                \
    if (!field) {                                                                                                      \
      return std::unexpected("Failed to load '" + std::string(_key) + "': " + field.error());                          \
    }                                                                                                                  \
    (_field) = *field;                                                                                                 \
  } while (false)

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

YAML::Node dumpMission(const Mission& mission)
{
  constexpr int kGnssPrecision = 12;

  YAML::Node mission_node(YAML::NodeType::Sequence);

  for (const auto& item : mission.items) {
    YAML::Node item_node(YAML::NodeType::Map);
    YAML::Node data_node(YAML::NodeType::Map);

    switch (item.type) {
      case Type::kWaypoint: {
        const auto waypoint = st::fromBytes<Waypoint>(item.data);
        item_node[kTypeKey] = kTypeWaypoint;
        data_node[kWaypointLatitude] = yaml::format(waypoint.latitude, kGnssPrecision);
        data_node[kWaypointLongitude] = yaml::format(waypoint.longitude, kGnssPrecision);
        data_node[kWaypointAltitude] = yaml::format(waypoint.altitude);
        data_node[kWaypointAltitudeFrame] = waypoint.altitude_frame;
        data_node[kWaypointAutoHeading] = waypoint.auto_heading;
        data_node[kWaypointStopAtWaypoint] = waypoint.stop_at_waypoint;
        data_node[kWaypointMaxHorizontalVelocity] = yaml::format(waypoint.max_horizontal_velocity);
        data_node[kWaypointMaxHorizontalAccel] = yaml::format(waypoint.max_horizontal_accel);
        data_node[kWaypointMaxHorizontalJerk] = yaml::format(waypoint.max_horizontal_jerk);
        data_node[kWaypointMaxVerticalVelocity] = yaml::format(waypoint.max_vertical_velocity);
        data_node[kWaypointMaxVerticalAccel] = yaml::format(waypoint.max_vertical_accel);
        data_node[kWaypointMaxVerticalJerk] = yaml::format(waypoint.max_vertical_jerk);
        data_node[kWaypointMaxHeadingRate] = yaml::format(waypoint.max_heading_rate);
        data_node[kWaypointMaxHeadingAccel] = yaml::format(waypoint.max_heading_accel);
        data_node[kWaypointAcceptanceRadius] = yaml::format(waypoint.acceptance_radius);
        data_node[kWaypointAltitudeTolerance] = yaml::format(waypoint.altitude_tolerance);
        data_node[kWaypointTimeout] = yaml::format(waypoint.timeout);
        break;
      }
      case Type::kTakeoff: {
        const auto takeoff = st::fromBytes<Takeoff>(item.data);
        item_node[kTypeKey] = kTypeTakeoff;
        data_node[kTakeoffAltitude] = yaml::format(takeoff.altitude);
        data_node[kTakeoffAltitudeFrame] = takeoff.altitude_frame;
        data_node[kTakeoffMaxSpeed] = yaml::format(takeoff.max_speed);
        data_node[kTakeoffMaxAccel] = yaml::format(takeoff.max_accel);
        data_node[kTakeoffMaxJerk] = yaml::format(takeoff.max_jerk);
        data_node[kTakeoffAltitudeTolerance] = yaml::format(takeoff.altitude_tolerance);
        data_node[kTakeoffTimeout] = yaml::format(takeoff.timeout);
        break;
      }
      case Type::kLand: {
        const auto land = st::fromBytes<Land>(item.data);
        item_node[kTypeKey] = kTypeLand;
        data_node[kLandSpeed] = yaml::format(land.speed);
        data_node[kLandTimeout] = yaml::format(land.timeout);
        break;
      }
      case Type::kReturnToLaunch: {
        const auto rtl = st::fromBytes<ReturnToLaunch>(item.data);
        item_node[kTypeKey] = kTypeRtl;
        data_node[kRtlMinAltitude] = yaml::format(rtl.min_altitude);
        data_node[kRtlMaxHorizontalVelocity] = yaml::format(rtl.max_horizontal_velocity);
        data_node[kRtlMaxHorizontalAccel] = yaml::format(rtl.max_horizontal_accel);
        data_node[kRtlMaxHorizontalJerk] = yaml::format(rtl.max_horizontal_jerk);
        data_node[kRtlMaxVerticalVelocity] = yaml::format(rtl.max_vertical_velocity);
        data_node[kRtlMaxVerticalAccel] = yaml::format(rtl.max_vertical_accel);
        data_node[kRtlMaxVerticalJerk] = yaml::format(rtl.max_vertical_jerk);
        data_node[kRtlMaxHeadingRate] = yaml::format(rtl.max_heading_rate);
        data_node[kRtlMaxHeadingAccel] = yaml::format(rtl.max_heading_accel);
        data_node[kRtlAcceptanceRadius] = yaml::format(rtl.acceptance_radius);
        data_node[kRtlAltitudeTolerance] = yaml::format(rtl.altitude_tolerance);
        data_node[kRtlTimeout] = yaml::format(rtl.timeout);
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

std::expected<Mission, std::string> loadMission(const YAML::Node& mission_node)
{
  Mission mission;
  if (!mission_node.IsSequence()) {
    return std::unexpected("Mission node must be a sequence.");
  }

  for (const auto& item_node : mission_node) {
    const auto type = yaml::load<std::string>(kTypeKey, item_node);
    if (!type) {
      return std::unexpected("Failed to load a mission type: " + type.error());
    }

    const auto data_node = item_node[kDataKey];
    if (!data_node.IsDefined()) {
      return std::unexpected("Mission item data is not defined.");
    }

    MissionItem item;

    if (*type == kTypeWaypoint) {
      Waypoint waypoint;
      LOAD_PACKED_FIELD(kWaypointLatitude, data_node, waypoint.latitude);
      LOAD_PACKED_FIELD(kWaypointLongitude, data_node, waypoint.longitude);
      LOAD_PACKED_FIELD(kWaypointAltitude, data_node, waypoint.altitude);
      LOAD_PACKED_FIELD(kWaypointAltitudeFrame, data_node, waypoint.altitude_frame);
      LOAD_PACKED_FIELD(kWaypointAutoHeading, data_node, waypoint.auto_heading);
      LOAD_PACKED_FIELD(kWaypointStopAtWaypoint, data_node, waypoint.stop_at_waypoint);
      LOAD_PACKED_FIELD(kWaypointMaxHorizontalVelocity, data_node, waypoint.max_horizontal_velocity);
      LOAD_PACKED_FIELD(kWaypointMaxHorizontalAccel, data_node, waypoint.max_horizontal_accel);
      LOAD_PACKED_FIELD(kWaypointMaxHorizontalJerk, data_node, waypoint.max_horizontal_jerk);
      LOAD_PACKED_FIELD(kWaypointMaxVerticalVelocity, data_node, waypoint.max_vertical_velocity);
      LOAD_PACKED_FIELD(kWaypointMaxVerticalAccel, data_node, waypoint.max_vertical_accel);
      LOAD_PACKED_FIELD(kWaypointMaxVerticalJerk, data_node, waypoint.max_vertical_jerk);
      LOAD_PACKED_FIELD(kWaypointMaxHeadingRate, data_node, waypoint.max_heading_rate);
      LOAD_PACKED_FIELD(kWaypointMaxHeadingAccel, data_node, waypoint.max_heading_accel);
      LOAD_PACKED_FIELD(kWaypointAcceptanceRadius, data_node, waypoint.acceptance_radius);
      LOAD_PACKED_FIELD(kWaypointAltitudeTolerance, data_node, waypoint.altitude_tolerance);
      LOAD_PACKED_FIELD(kWaypointTimeout, data_node, waypoint.timeout);
      item.type = Type::kWaypoint;
      item.data = st::toBytes(waypoint);
    }
    else if (*type == kTypeTakeoff) {
      Takeoff takeoff;
      LOAD_PACKED_FIELD(kTakeoffAltitude, data_node, takeoff.altitude);
      LOAD_PACKED_FIELD(kTakeoffAltitudeFrame, data_node, takeoff.altitude_frame);
      LOAD_PACKED_FIELD(kTakeoffMaxSpeed, data_node, takeoff.max_speed);
      LOAD_PACKED_FIELD(kTakeoffMaxAccel, data_node, takeoff.max_accel);
      LOAD_PACKED_FIELD(kTakeoffMaxJerk, data_node, takeoff.max_jerk);
      LOAD_PACKED_FIELD(kTakeoffAltitudeTolerance, data_node, takeoff.altitude_tolerance);
      LOAD_PACKED_FIELD(kTakeoffTimeout, data_node, takeoff.timeout);
      item.type = Type::kTakeoff;
      item.data = st::toBytes(takeoff);
    }
    else if (*type == kTypeLand) {
      Land land;
      LOAD_PACKED_FIELD(kLandSpeed, data_node, land.speed);
      LOAD_PACKED_FIELD(kLandTimeout, data_node, land.timeout);
      item.type = Type::kLand;
      item.data = st::toBytes(land);
    }
    else if (*type == kTypeRtl) {
      ReturnToLaunch rtl;
      LOAD_PACKED_FIELD(kRtlMinAltitude, data_node, rtl.min_altitude);
      LOAD_PACKED_FIELD(kRtlMaxHorizontalVelocity, data_node, rtl.max_horizontal_velocity);
      LOAD_PACKED_FIELD(kRtlMaxHorizontalAccel, data_node, rtl.max_horizontal_accel);
      LOAD_PACKED_FIELD(kRtlMaxHorizontalJerk, data_node, rtl.max_horizontal_jerk);
      LOAD_PACKED_FIELD(kRtlMaxVerticalVelocity, data_node, rtl.max_vertical_velocity);
      LOAD_PACKED_FIELD(kRtlMaxVerticalAccel, data_node, rtl.max_vertical_accel);
      LOAD_PACKED_FIELD(kRtlMaxVerticalJerk, data_node, rtl.max_vertical_jerk);
      LOAD_PACKED_FIELD(kRtlMaxHeadingRate, data_node, rtl.max_heading_rate);
      LOAD_PACKED_FIELD(kRtlMaxHeadingAccel, data_node, rtl.max_heading_accel);
      LOAD_PACKED_FIELD(kRtlAcceptanceRadius, data_node, rtl.acceptance_radius);
      LOAD_PACKED_FIELD(kRtlAltitudeTolerance, data_node, rtl.altitude_tolerance);
      LOAD_PACKED_FIELD(kRtlTimeout, data_node, rtl.timeout);
      item.type = Type::kReturnToLaunch;
      item.data = st::toBytes(rtl);
    }
    else {
      return std::unexpected("Invalid mission item type: " + *type);
    }

    mission.items.push_back(item);
  }

  return mission;
}
}  // namespace mission
}  // namespace tobas
