uint8 mode
geometry_msgs/Vector3 target_position   # [m]
geometry_msgs/Vector3 target_velocity   # [m/s]
float64 target_yaw_angle                # [rad]
float64 target_yaw_rate                 # [rad/s]

uint8 GLOBAL_POSITION = 1
uint8 GLOBAL_VELOCITY = 2
uint8 LOCAL_VELOCITY = 3
