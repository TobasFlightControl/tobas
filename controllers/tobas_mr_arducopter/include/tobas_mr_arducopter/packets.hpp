#pragma once

#include "./constants.hpp"

namespace tobas_mr_arducopter
{
/// \brief A servo packet.
struct ServoPacket
{
  /// \brief Motor speed data.
  /// should rename to servo_command here and in ArduPilot SIM_Gazebo.cpp
  float motorSpeed[kMaxMotors] = { 0.0f };
};

/// \brief Flight Dynamics Model packet that is sent back to the ArduPilot
struct FdmPacket
{
  /// \brief packet timestamp
  double timestamp;

  /// \brief IMU angular velocity
  double imuAngularVelocityRPY[3];

  /// \brief IMU linear acceleration
  double imuLinearAccelerationXYZ[3];

  /// \brief IMU quaternion orientation
  double imuOrientationQuat[4];

  /// \brief Model velocity in NED frame
  double velocityXYZ[3];

  /// \brief Model position in NED frame
  double positionXYZ[3];
  /*  NOT MERGED IN MASTER YET
    /// \brief Model latitude in WGS84 system
    double latitude = 0.0;

    /// \brief Model longitude in WGS84 system
    double longitude = 0.0;

    /// \brief Model altitude from GPS
    double altitude = 0.0;

    /// \brief Model estimated from airspeed sensor (e.g. Pitot) in m/s
    double airspeed = 0.0;

    /// \brief Battery voltage. Default to -1 to use sitl estimator.
    double battery_voltage = -1.0;

    /// \brief Battery Current.
    double battery_current = 0.0;

    /// \brief Model rangefinder value. Default to -1 to use sitl rangefinder.
    double rangefinder = -1.0;
  */
};
}  // namespace tobas_mr_arducopter
