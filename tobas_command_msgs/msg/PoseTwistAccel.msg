std_msgs/Header header
tobas_command_msgs/CommandLevel level

tobas_kdl_msgs/Vector pos    # Target position wrt. the global coordinates [m]
tobas_kdl_msgs/Vector vel    # Target linear velocity wrt. the global coordinates [m/s]
tobas_kdl_msgs/Vector acc    # Target linear acceleration wrt. the global coordinates [m/s^2]

tobas_kdl_msgs/Euler rpy     # Target orientation wrt. the global coordinates [rad]
tobas_kdl_msgs/Vector gyro   # Target angular velocity wrt. the local coordinates [rad/s]
tobas_kdl_msgs/Vector dgyro  # Target angular acceleration wrt. the local coordinates [rad/s^2]
