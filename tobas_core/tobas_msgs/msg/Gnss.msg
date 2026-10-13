# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header

# Fix Type
# Larger values mean better fixes. RTK_FLOAT and RTK_FIX are 3D fixes with an RTK solution.
# Dead reckoning and time-only fixes are regarded as NO_FIX.
uint8 fix_type
uint8 NO_FIX = 0
uint8 FIX_2D = 1
uint8 FIX_3D = 2
uint8 RTK_FLOAT = 3
uint8 RTK_FIX = 4

# Position
float64 latitude                               # Geodetic latitude [deg]
float64 longitude                              # Geodetic longitude [deg]
float64 height_wgs84                           # Height above the WGS 84 ellipsoid [m]
float64 height_msl                             # Height above mean sea level [m]
tobas_eigen_msgs/Matrix3d position_covariance  # Position covariance [m^2]

# Velocity
tobas_kdl_msgs/Vector ground_speed             # Ground velocity in ENU coordinates [m/s]
tobas_eigen_msgs/Matrix3d velocity_covariance  # Ground-velocity covariance [m^2/s^2]

# Status
uint8 num_satellites_used  # Satellites used in the navigation solution
