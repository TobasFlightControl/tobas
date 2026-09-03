# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header

# Fix Type: gpsFix (UBX-STATUS), fixType (UBX-PVT)
uint8 fix_type
uint8 NO_FIX = 0
uint8 DEAD_RECHONING_ONLY = 1
uint8 FIX_2D = 2
uint8 FIX_3D = 3
uint8 GPS_DEAD_RECHONING_COMBINED = 4
uint8 TIME_ONLY_FIX = 5

uint8 rtk_status
uint8 STAND_ALONE = 0
uint8 CORRECTION_RECEIVED = 1
uint8 RTK_FLOAT = 2
uint8 RTK_FIXED = 3

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
