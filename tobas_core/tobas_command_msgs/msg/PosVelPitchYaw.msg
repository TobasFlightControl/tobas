std_msgs/Header header
tobas_command_msgs/Priority priority

tobas_kdl_msgs/Vector pos  # Target position wrt. the global coordinates [m]
tobas_kdl_msgs/Vector vel  # Target linear velocity wrt. the global coordinates [m/s]
float64 pitch              # Target pitch angle [rad]
float64 yaw                # Target yaw angle [rad]
