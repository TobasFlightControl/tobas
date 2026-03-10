std_msgs/Header header

tobas_kdl_msgs/Vector target_position  # Target position expressed in the global frame [m]
tobas_kdl_msgs/Vector target_velocity  # Target linear velocity expressed in the body frame [m]
tobas_kdl_msgs/Vector target_accel     # Target linear acceleration expressed in the body frame [m]
tobas_kdl_msgs/Euler target_angle      # Target orientation expressed in the global frame [rad]
tobas_kdl_msgs/Vector target_gyro      # Target angular velocity expressed in the body frame [rad/s]
tobas_kdl_msgs/Vector target_dgyro     # Target angular acceleration expressed in the body frame [rad/s]

tobas_kdl_msgs/Vector position_integral_error  # Position integral error [m s]
tobas_kdl_msgs/Vector angle_integral_error     # Orientation integral error [m s]
