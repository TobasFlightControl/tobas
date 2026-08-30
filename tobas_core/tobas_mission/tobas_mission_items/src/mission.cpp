// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_mission_items/mission.hpp"

#include <tobas_std_tools/byte.hpp>
#include <tobas_yaml_tools/convert/float64.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

#include "tobas_mission_items/mission_items.hpp"

// Mission item elements
#define TYPE_KEY "type"
#define DATA_KEY "data"

// Mission item types
#define TYPE_WAYPOINT "waypoint"
#define TYPE_TAKEOFF "takeoff"
#define TYPE_LAND "land"
#define TYPE_RTL "rtl"

// Waypoint elements
#define WAYPOINT_LATITUDE "latitude"
#define WAYPOINT_LONGITUDE "longitude"
#define WAYPOINT_ALTITUDE "altitude"
#define WAYPOINT_ALTITUDE_FRAME "altitude_frame"
#define WAYPOINT_AUTO_HEADING "auto_heading"
#define WAYPOINT_STOP_AT_WAYPOINT "stop_at_waypoint"
#define WAYPOINT_MAX_HORIZONTAL_VELOCITY "max_horizontal_velocity"
#define WAYPOINT_MAX_HORIZONTAL_ACCEL "max_horizontal_accel"
#define WAYPOINT_MAX_HORIZONTAL_JERK "max_horizontal_jerk"
#define WAYPOINT_MAX_VERTICAL_VELOCITY "max_vertical_velocity"
#define WAYPOINT_MAX_VERTICAL_ACCEL "max_vertical_accel"
#define WAYPOINT_MAX_VERTICAL_JERK "max_vertical_jerk"
#define WAYPOINT_MAX_HEADING_RATE "max_heading_rate"
#define WAYPOINT_MAX_HEADING_ACCEL "max_heading_accel"
#define WAYPOINT_ACCEPTANCE_RADIUS "acceptance_radius"
#define WAYPOINT_ALTITUDE_TOLERANCE "altitude_tolerance"
#define WAYPOINT_TIMEOUT "timeout"

// Takeoff elements
#define TAKEOFF_ALTITUDE "altitude"
#define TAKEOFF_ALTITUDE_FRAME "altitude_frame"
#define TAKEOFF_MAX_SPEED "max_speed"
#define TAKEOFF_MAX_ACCEL "max_accel"
#define TAKEOFF_MAX_JERK "max_jerk"
#define TAKEOFF_ALTITUDE_TOLERANCE "altitude_tolerance"
#define TAKEOFF_TIMEOUT "timeout"

// Land elements
#define LAND_SPEED "speed"
#define LAND_TIMEOUT "timeout"

// RTL elements
#define RTL_MIN_ALTITUDE "min_altitude"
#define RTL_ALTITUDE_FRAME "altitude_frame"
#define RTL_MAX_HORIZONTAL_VELOCITY "max_horizontal_velocity"
#define RTL_MAX_HORIZONTAL_ACCEL "max_horizontal_accel"
#define RTL_MAX_HORIZONTAL_JERK "max_horizontal_jerk"
#define RTL_MAX_VERTICAL_VELOCITY "max_vertical_velocity"
#define RTL_MAX_VERTICAL_ACCEL "max_vertical_accel"
#define RTL_MAX_VERTICAL_JERK "max_vertical_jerk"
#define RTL_MAX_HEADING_RATE "max_heading_rate"
#define RTL_MAX_HEADING_ACCEL "max_heading_accel"
#define RTL_ACCEPTANCE_RADIUS "acceptance_radius"
#define RTL_ALTITUDE_TOLERANCE "altitude_tolerance"
#define RTL_TIMEOUT "timeout"

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
        item_node[TYPE_KEY] = TYPE_WAYPOINT;
        data_node[WAYPOINT_LATITUDE] = tobas::yaml::format(waypoint.latitude, kGnssPrecision);
        data_node[WAYPOINT_LONGITUDE] = tobas::yaml::format(waypoint.longitude, kGnssPrecision);
        data_node[WAYPOINT_ALTITUDE] = tobas::yaml::format(waypoint.altitude);
        data_node[WAYPOINT_ALTITUDE_FRAME] = waypoint.altitude_frame;
        data_node[WAYPOINT_AUTO_HEADING] = waypoint.auto_heading;
        data_node[WAYPOINT_STOP_AT_WAYPOINT] = waypoint.stop_at_waypoint;
        data_node[WAYPOINT_MAX_HORIZONTAL_VELOCITY] = tobas::yaml::format(waypoint.max_horizontal_velocity);
        data_node[WAYPOINT_MAX_HORIZONTAL_ACCEL] = tobas::yaml::format(waypoint.max_horizontal_accel);
        data_node[WAYPOINT_MAX_HORIZONTAL_JERK] = tobas::yaml::format(waypoint.max_horizontal_jerk);
        data_node[WAYPOINT_MAX_VERTICAL_VELOCITY] = tobas::yaml::format(waypoint.max_vertical_velocity);
        data_node[WAYPOINT_MAX_VERTICAL_ACCEL] = tobas::yaml::format(waypoint.max_vertical_accel);
        data_node[WAYPOINT_MAX_VERTICAL_JERK] = tobas::yaml::format(waypoint.max_vertical_jerk);
        data_node[WAYPOINT_MAX_HEADING_RATE] = tobas::yaml::format(waypoint.max_heading_rate);
        data_node[WAYPOINT_MAX_HEADING_ACCEL] = tobas::yaml::format(waypoint.max_heading_accel);
        data_node[WAYPOINT_ACCEPTANCE_RADIUS] = tobas::yaml::format(waypoint.acceptance_radius);
        data_node[WAYPOINT_ALTITUDE_TOLERANCE] = tobas::yaml::format(waypoint.altitude_tolerance);
        data_node[WAYPOINT_TIMEOUT] = tobas::yaml::format(waypoint.timeout);
        break;
      }
      case tobas::mission::Type::kTakeoff: {
        tobas::mission::Takeoff takeoff;
        if (!tobas::st::fromBytes(item.data, takeoff)) {
          std::cerr << "Failed to decode a takeoff mission." << std::endl;
          continue;
        }
        item_node[TYPE_KEY] = TYPE_TAKEOFF;
        data_node[TAKEOFF_ALTITUDE] = tobas::yaml::format(takeoff.altitude);
        data_node[TAKEOFF_ALTITUDE_FRAME] = takeoff.altitude_frame;
        data_node[TAKEOFF_MAX_SPEED] = tobas::yaml::format(takeoff.max_speed);
        data_node[TAKEOFF_MAX_ACCEL] = tobas::yaml::format(takeoff.max_accel);
        data_node[TAKEOFF_MAX_JERK] = tobas::yaml::format(takeoff.max_jerk);
        data_node[TAKEOFF_ALTITUDE_TOLERANCE] = tobas::yaml::format(takeoff.altitude_tolerance);
        data_node[TAKEOFF_TIMEOUT] = tobas::yaml::format(takeoff.timeout);
        break;
      }
      case tobas::mission::Type::kLand: {
        tobas::mission::Land land;
        if (!tobas::st::fromBytes(item.data, land)) {
          std::cerr << "Failed to decode a land mission." << std::endl;
          continue;
        }
        item_node[TYPE_KEY] = TYPE_LAND;
        data_node[LAND_SPEED] = tobas::yaml::format(land.speed);
        data_node[LAND_TIMEOUT] = tobas::yaml::format(land.timeout);
        break;
      }
      case tobas::mission::Type::kReturnToLaunch: {
        tobas::mission::ReturnToLaunch rtl;
        if (!tobas::st::fromBytes(item.data, rtl)) {
          std::cerr << "Failed to decode a RTL mission." << std::endl;
          continue;
        }
        item_node[TYPE_KEY] = TYPE_RTL;
        data_node[RTL_MIN_ALTITUDE] = tobas::yaml::format(rtl.min_altitude);
        data_node[RTL_MAX_HORIZONTAL_VELOCITY] = tobas::yaml::format(rtl.max_horizontal_velocity);
        data_node[RTL_MAX_HORIZONTAL_ACCEL] = tobas::yaml::format(rtl.max_horizontal_accel);
        data_node[RTL_MAX_HORIZONTAL_JERK] = tobas::yaml::format(rtl.max_horizontal_jerk);
        data_node[RTL_MAX_VERTICAL_VELOCITY] = tobas::yaml::format(rtl.max_vertical_velocity);
        data_node[RTL_MAX_VERTICAL_ACCEL] = tobas::yaml::format(rtl.max_vertical_accel);
        data_node[RTL_MAX_VERTICAL_JERK] = tobas::yaml::format(rtl.max_vertical_jerk);
        data_node[RTL_MAX_HEADING_RATE] = tobas::yaml::format(rtl.max_heading_rate);
        data_node[RTL_MAX_HEADING_ACCEL] = tobas::yaml::format(rtl.max_heading_accel);
        data_node[RTL_ACCEPTANCE_RADIUS] = tobas::yaml::format(rtl.acceptance_radius);
        data_node[RTL_ALTITUDE_TOLERANCE] = tobas::yaml::format(rtl.altitude_tolerance);
        data_node[RTL_TIMEOUT] = tobas::yaml::format(rtl.timeout);
        break;
      }
      default:
        throw;
    }

    item_node[DATA_KEY] = data_node;
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
    const auto type = tobas::yaml::load<std::string>(TYPE_KEY, item_node);
    if (!type) {
      std::cerr << type.error() << std::endl;
      return false;
    }

    const auto data_node = item_node[DATA_KEY];
    if (!data_node.IsDefined()) {
      std::cerr << "\"" << DATA_KEY << "\" is not defined." << std::endl;
      return false;
    }

    tobas::mission::MissionItem item;

    if (*type == TYPE_WAYPOINT) {
      tobas::mission::Waypoint waypoint;
      if (!LOAD_PACKED_FIELD(WAYPOINT_LATITUDE, data_node, waypoint.latitude)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_LONGITUDE, data_node, waypoint.longitude)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_ALTITUDE, data_node, waypoint.altitude)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_ALTITUDE_FRAME, data_node, waypoint.altitude_frame)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_AUTO_HEADING, data_node, waypoint.auto_heading)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_STOP_AT_WAYPOINT, data_node, waypoint.stop_at_waypoint)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_MAX_HORIZONTAL_VELOCITY, data_node, waypoint.max_horizontal_velocity)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_MAX_HORIZONTAL_ACCEL, data_node, waypoint.max_horizontal_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_MAX_HORIZONTAL_JERK, data_node, waypoint.max_horizontal_jerk)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_MAX_VERTICAL_VELOCITY, data_node, waypoint.max_vertical_velocity)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_MAX_VERTICAL_ACCEL, data_node, waypoint.max_vertical_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_MAX_VERTICAL_JERK, data_node, waypoint.max_vertical_jerk)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_MAX_HEADING_RATE, data_node, waypoint.max_heading_rate)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_MAX_HEADING_ACCEL, data_node, waypoint.max_heading_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_ACCEPTANCE_RADIUS, data_node, waypoint.acceptance_radius)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_ALTITUDE_TOLERANCE, data_node, waypoint.altitude_tolerance)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(WAYPOINT_TIMEOUT, data_node, waypoint.timeout)) {
        return false;
      }
      item.type = tobas::mission::Type::kWaypoint;
      item.data = tobas::st::toBytes(waypoint);
    }
    else if (*type == TYPE_TAKEOFF) {
      tobas::mission::Takeoff takeoff;
      if (!LOAD_PACKED_FIELD(TAKEOFF_ALTITUDE, data_node, takeoff.altitude)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(TAKEOFF_ALTITUDE_FRAME, data_node, takeoff.altitude_frame)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(TAKEOFF_MAX_SPEED, data_node, takeoff.max_speed)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(TAKEOFF_MAX_ACCEL, data_node, takeoff.max_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(TAKEOFF_MAX_JERK, data_node, takeoff.max_jerk)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(TAKEOFF_ALTITUDE_TOLERANCE, data_node, takeoff.altitude_tolerance)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(TAKEOFF_TIMEOUT, data_node, takeoff.timeout)) {
        return false;
      }
      item.type = tobas::mission::Type::kTakeoff;
      item.data = tobas::st::toBytes(takeoff);
    }
    else if (*type == TYPE_LAND) {
      tobas::mission::Land land;
      if (!LOAD_PACKED_FIELD(LAND_SPEED, data_node, land.speed)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(LAND_TIMEOUT, data_node, land.timeout)) {
        return false;
      }
      item.type = tobas::mission::Type::kLand;
      item.data = tobas::st::toBytes(land);
    }
    else if (*type == TYPE_RTL) {
      tobas::mission::ReturnToLaunch rtl;
      if (!LOAD_PACKED_FIELD(RTL_MIN_ALTITUDE, data_node, rtl.min_altitude)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(RTL_MAX_HORIZONTAL_VELOCITY, data_node, rtl.max_horizontal_velocity)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(RTL_MAX_HORIZONTAL_ACCEL, data_node, rtl.max_horizontal_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(RTL_MAX_HORIZONTAL_JERK, data_node, rtl.max_horizontal_jerk)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(RTL_MAX_VERTICAL_VELOCITY, data_node, rtl.max_vertical_velocity)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(RTL_MAX_VERTICAL_ACCEL, data_node, rtl.max_vertical_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(RTL_MAX_VERTICAL_JERK, data_node, rtl.max_vertical_jerk)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(RTL_MAX_HEADING_RATE, data_node, rtl.max_heading_rate)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(RTL_MAX_HEADING_ACCEL, data_node, rtl.max_heading_accel)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(RTL_ACCEPTANCE_RADIUS, data_node, rtl.acceptance_radius)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(RTL_ALTITUDE_TOLERANCE, data_node, rtl.altitude_tolerance)) {
        return false;
      }
      if (!LOAD_PACKED_FIELD(RTL_TIMEOUT, data_node, rtl.timeout)) {
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
