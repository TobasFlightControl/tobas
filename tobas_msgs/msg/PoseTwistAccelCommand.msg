tobas_msgs/CommandLevel level
tobas_msgs/FrameId frame_id  # The frame in whch velocity and acceleration are expressed

tobas_kdl_msgs/Vector pos    # Target position [m]
tobas_kdl_msgs/Vector vel    # Target linear velocity [m/s]
tobas_kdl_msgs/Vector acc    # Target linear acceleration (feedforward) [m/s^2]

tobas_kdl_msgs/Euler rpy     # Target orientation [rad]
tobas_kdl_msgs/Vector gyro   # Target angular velocity wrt. the local coordinates [rad/s]
tobas_kdl_msgs/Vector dgyro  # Target angular acceleration wrt. the local coordinates (feedforward) [rad/s^2]
