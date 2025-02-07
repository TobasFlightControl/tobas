std_msgs/Header header

tobas_command_msgs/CommandLevel level
tobas_command_msgs/FrameId frame_id  # The frame in whch velocity and acceleration are expressed

tobas_kdl_msgs/Vector pos    # [m]
tobas_kdl_msgs/Vector vel    # [m/s]
tobas_kdl_msgs/Vector acc    # [m/s^2]
float64 yaw                  # [rad]
