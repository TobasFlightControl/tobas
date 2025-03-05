std_msgs/Header header

tobas_kdl_msgs/Vector target_position      # Target position wrt. the global coordinates [m]
tobas_kdl_msgs/Vector target_velocity      # Target velocity wrt. the global coordinates [m]
tobas_kdl_msgs/Vector target_acceleration  # Target acceleration wrt. the global coordinates [m]
tobas_kdl_msgs/Euler target_angle          # Target euler angle wrt. the global coordinates [rad]
tobas_kdl_msgs/Vector target_rate          # Target gyro wrt. the local coordinates [rad/s]
float64 target_thrust                      # Target thrust force [N]

tobas_kdl_msgs/Vector position_integral_error  # Position integral error [m s]
