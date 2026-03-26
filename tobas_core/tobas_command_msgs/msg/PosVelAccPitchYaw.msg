std_msgs/Header header
tobas_command_msgs/Priority priority

tobas_kdl_msgs/Vector pos  # Target position expressed in the global frame [m]
tobas_kdl_msgs/Vector vel  # Target linear velocity expressed in the global frame [m/s]
tobas_kdl_msgs/Vector acc  # Target linear acceleration expressed in the global frame [m/s]
float64 pitch              # Target pitch angle [rad]
float64 yaw                # Target yaw angle [rad]
