std_msgs/Header header
tobas_command_msgs/CommandLevel level

tobas_kdl_msgs/Vector pos  # Target position wrt. the global coordinates [m]
tobas_kdl_msgs/Vector vel  # Target linear velocity wrt. the global coordinates [m/s]
tobas_kdl_msgs/Vector acc  # Target linear acceleration wrt. the global coordinates [m/s^2]
float64 yaw                # Target yaw angle [rad]
