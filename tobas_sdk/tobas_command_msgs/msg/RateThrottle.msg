std_msgs/Header header
tobas_command_msgs/CommandLevel level

tobas_kdl_msgs/Vector rate  # Target angular velocity wrt. the local coordinates [rad/s]
float64 throttle            # Target throttle [0, 1]
