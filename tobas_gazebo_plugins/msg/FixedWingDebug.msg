std_msgs/Header header
kdl_msgs/Vector relative_body_velocity  # uvw (NED body frame) [m/s]
float64 alpha                           # Angle of attack [rad]
float64 beta                            # Angle of sideslip [rad]
kdl_msgs/Vector air_force               # Air force (NWU body frame) [N]
kdl_msgs/Vector air_moment              # Air moment (NWU body frame) [Nm]
float64[] deflections                   # deflection angles of control surfaces [rad]
