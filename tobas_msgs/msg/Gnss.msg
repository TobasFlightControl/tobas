std_msgs/Header header

# Fix Type: gnssFix (UBX-STATUS), fixType (UBX-PVT)
uint8 fix_type
uint8 NO_FIX = 0
uint8 DEAD_RECHONING_ONLY = 1
uint8 FIX_2D = 2
uint8 FIX_3D = 3
uint8 GPS_DEAD_RECHONING_COMBINED = 4
uint8 TIME_ONLY_FIX = 5

# Position
float64 latitude                                # [deg]
float64 longitude                               # [deg]
float64 altitude                                # [m]
tobas_eigen_msgs/Matrix3d position_covariance   # [m^2]

# Velocity
tobas_kdl_msgs/Vector ground_speed                 # [m/s]
tobas_eigen_msgs/Matrix3d velocity_covariance      # [m^2/s^2]
