std_msgs/Header header
geometry_msgs/Vector3 relative_body_velocity  # uvw (NED body frame) [m/s]
float64 alpha                                 # Angle of attack [rad]
float64 beta                                  # Angle of sideslip [rad]
geometry_msgs/Vector3 air_force               # Air force (NWU body frame) [N]
geometry_msgs/Vector3 air_moment              # Air moment (NWU body frame) [Nm]
float64[] deflections                         # deflection angles of control surfaces [rad]
