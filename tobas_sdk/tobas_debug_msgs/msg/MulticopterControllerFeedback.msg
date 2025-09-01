std_msgs/Header header

tobas_kdl_msgs/Vector target_position  # Target position wrt. the global coordinates [m]
tobas_kdl_msgs/Vector target_velocity  # Target linear velocity wrt. the local coordinates [m]
tobas_kdl_msgs/Vector target_accel     # Target linear acceleration wrt. the local coordinates [m]
tobas_kdl_msgs/Euler target_angle      # Target orientation wrt. the global coordinates [rad]
tobas_kdl_msgs/Vector target_gyro      # Target angular velocity wrt. the local coordinates [rad/s]
tobas_kdl_msgs/Vector target_dgyro     # Target angular acceleration wrt. the local coordinates [rad/s]

tobas_kdl_msgs/Vector position_integral_error  # Position integral error [m s]
tobas_kdl_msgs/Vector angle_integral_error     # Orientation integral error [m s]
