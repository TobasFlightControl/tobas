# ROS API

This section describes the main topics, services, and actions provided by Tobas.
The available APIs depend on the aircraft configuration and the nodes running. Check them at runtime using `ros2 topic list`, `ros2 service list`, and `ros2 action list`.
When communicating with the FC from an external terminal, use the APIs exposed in the `remote_interface` namespace.

## Topics

<!-- tobas_constants/ros_interfaces.hppの内容 -->
<!-- tobas_gazebo_common/constants.hppの内容 -->

---

### Common

These topics are available on both real hardware and in simulation.

#### battery (tobas_msgs/Battery)

Publishes the battery voltage [V] and current [A].

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 voltage  # [V]
float64 current  # [A]
```

#### engine_state (tobas_msgs/EngineState)

Publishes the engine rotational speed [rad/s], remaining fuel [L], and oil temperature [degC].

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 speed            # [rad/s]
float64 fuel_quantity    # [L]
float64 oil_temperature  # [degC]
```

#### cpu (tobas_msgs/Cpu)

Publishes the FC CPU frequency [Hz], temperature [degC], and load.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
uint64 frequency     # [Hz]
float64 temperature  # [degC]
float64 load         # [-]
```

#### sbus (tobas_msgs/Sbus)

Publishes S.BUS channel values and reception status from the RC receiver.
`frame_lost` indicates frame loss, and `failsafe` indicates the receiver's failsafe status.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
uint16[16] periods
bool ch17
bool ch18
bool frame_lost
bool failsafe
```

#### rc_input (tobas_msgs/RCInput)

Converts RC input into flight control values and publishes them.
Includes roll, pitch, throttle, and yaw inputs, as well as the flight mode, control enabled/disabled status, kill switch status, and general-purpose switch status.
Use `status` to check the input reception status.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

uint8 status
uint8 STATUS_OK = 0
uint8 STATUS_FRAME_LOST = 1
uint8 STATUS_TIMEOUT = 2
uint8 STATUS_OTHER = 3

float64 roll      # CH1: [-1, 1]
float64 pitch     # CH2: [-1, 1]
float64 throttle  # CH3: [-1, 1]
float64 yaw       # CH4: [-1, 1]
uint8 mode        # CH5: Flight Mode
bool sub_mode     # CH6: Sub Flight Mode
bool enable       # CH7: Enable Radio Control
bool kill         # CH8: Kill Switch
bool[8] gpsw      # CH9-16: General Purpose Switch
```

#### imu_raw (tobas_msgs/Imu)

Publishes unfiltered IMU data.
`accel` represents acceleration [m/s^2], `gyro` angular velocity [rad/s], and `dgyro` angular acceleration [rad/s^2].

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_kdl_msgs/Vector accel  # [m/s^2]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector gyro   # [rad/s]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector dgyro  # [rad/s^2]
	float64 x
	float64 y
	float64 z
```

#### imu_filtered (tobas_msgs/Imu)

Publishes filtered IMU data.
Uses the same format as `imu_raw` and includes acceleration, angular velocity, and angular acceleration.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_kdl_msgs/Vector accel  # [m/s^2]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector gyro   # [rad/s]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector dgyro  # [rad/s^2]
	float64 x
	float64 y
	float64 z
```

#### magnetic_field (tobas_msgs/MagneticField)

Publishes the three-axis magnetic field vector measured by the magnetic sensor.
Each component of `mag` is treated as dimensionless.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_kdl_msgs/Vector mag  # [-]
	float64 x
	float64 y
	float64 z
```

#### air_pressure (tobas_msgs/FluidPressure)

Publishes the atmospheric pressure measured by the pressure sensor in Pa.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 pressure  # [Pa]
```

#### gnss (tobas_msgs/Gnss)

Publishes GNSS position and velocity measurements and positioning status.
Position is expressed as latitude, longitude, height above the WGS 84 ellipsoid, and height above mean sea level. Ground velocity is expressed in the ENU frame.
Also includes position and velocity covariances, the fix type, and the number of satellites used for positioning.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# Fix Type: gpsFix (UBX-STATUS), fixType (UBX-PVT)
uint8 fix_type
uint8 NO_FIX = 0
uint8 DEAD_RECHONING_ONLY = 1
uint8 FIX_2D = 2
uint8 FIX_3D = 3
uint8 GPS_DEAD_RECHONING_COMBINED = 4
uint8 TIME_ONLY_FIX = 5

# Position
float64 latitude                               # Geodetic latitude [deg]
float64 longitude                              # Geodetic longitude [deg]
float64 height_wgs84                           # Height above the WGS 84 ellipsoid [m]
float64 height_msl                             # Height above mean sea level [m]
tobas_eigen_msgs/Matrix3d position_covariance  # Position covariance [m^2]
	float64[9] data

# Velocity
tobas_kdl_msgs/Vector ground_speed             # Ground velocity in ENU coordinates [m/s]
	float64 x
	float64 y
	float64 z
tobas_eigen_msgs/Matrix3d velocity_covariance  # Ground-velocity covariance [m^2/s^2]
	float64[9] data

# Status
uint8 num_satellites_used  # Satellites used in the navigation solution
```

#### rotor_states (tobas_msgs/RotorStateArray)

Publishes the rotational speed, thrust, and error status of each rotor.
Each element of `states` identifies a rotor using `link_name`.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_msgs/RotorState[] states
	string link_name
	float64 speed   # [rad/s]
	float64 thrust  # [N]
	uint8 status
	uint8 NO_ERROR = 0
	uint8 COMMUNICATION_FAILURE = 1
```

#### joint_states_2 (tobas_msgs/JointStateArray)

Publishes the position, velocity, and force or torque of each movable joint.
Each element of `states` identifies a joint using `name`.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_msgs/JointState[] states
	string name
	float64 position
	float64 velocity
	float64 effort
```

#### odom (tobas_msgs/OdometryWithCovarianceStamped)

Publishes the aircraft's position, attitude, linear and angular velocities, and linear and angular accelerations estimated by the state estimator.
Position and attitude are expressed in the global frame; velocities and accelerations are expressed in the body frame.
Also includes estimation error covariances for position, attitude, linear velocity, and angular velocity.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_msgs/OdometryWithCovariance odom
	tobas_msgs/Odometry odom
		tobas_kdl_msgs/Frame frame  # The transformation from the global frame to the body frame
			tobas_kdl_msgs/Vector trans
				float64 x
				float64 y
				float64 z
			tobas_kdl_msgs/Rotation rot
				float64[9] data
		tobas_kdl_msgs/Twist twist  # The 6D twist expressed in the body frame
			tobas_kdl_msgs/Vector linear
				float64 x
				float64 y
				float64 z
			tobas_kdl_msgs/Vector angular
				float64 x
				float64 y
				float64 z
		tobas_kdl_msgs/Accel accel  # The 6D accel expressed in the body frame
			tobas_kdl_msgs/Vector linear
				float64 x
				float64 y
				float64 z
			tobas_kdl_msgs/Vector angular
				float64 x
				float64 y
				float64 z
	tobas_eigen_msgs/Matrix3d position_covariance     # [m^2]
		float64[9] data
	tobas_eigen_msgs/Matrix3d orientation_covariance  # [rad^2]
		float64[9] data
	tobas_eigen_msgs/Matrix3d velocity_covariance     # [m^2/s^2]
		float64[9] data
	tobas_eigen_msgs/Matrix3d gyro_covariance         # [rad^2/s^2]
		float64[9] data
```

#### trajectory_setpoint (tobas_msgs/OdometryStamped)

Publishes the position, attitude, velocity, and acceleration setpoints currently being tracked by the controller.
Components not under control are set to NaN. For example, in attitude control mode, the position and velocity setpoints are NaN.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_msgs/Odometry odom
	tobas_kdl_msgs/Frame frame  # The transformation from the global frame to the body frame
		tobas_kdl_msgs/Vector trans
			float64 x
			float64 y
			float64 z
		tobas_kdl_msgs/Rotation rot
			float64[9] data
	tobas_kdl_msgs/Twist twist  # The 6D twist expressed in the body frame
		tobas_kdl_msgs/Vector linear
			float64 x
			float64 y
			float64 z
		tobas_kdl_msgs/Vector angular
			float64 x
			float64 y
			float64 z
	tobas_kdl_msgs/Accel accel  # The 6D accel expressed in the body frame
		tobas_kdl_msgs/Vector linear
			float64 x
			float64 y
			float64 z
		tobas_kdl_msgs/Vector angular
			float64 x
			float64 y
			float64 z
```

#### arming (tobas_msgs/Arming)

Publishes the rotor arming status.
If `data` is `true`, the rotors are armed; if it is `false`, they are disarmed.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
bool data
```

### Command

Publish commands to these topics to specify setpoints for the aircraft or its joints.
The available commands depend on the airframe type and flight mode. Use the ROS 2 CLI at runtime to check the target topics and subscribing nodes.
For commands with `priority`, specify `NORMAL` for normal commands, `DEFENSIVE` for defensive commands, and `MANUAL` for manual commands.

#### command/rate (tobas_command_msgs/Rate)

Commands target angular velocities about the three axes of the body frame in rad/s.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector rate  # Target angular velocity expressed in the local frame [rad/s]
	float64 x
	float64 y
	float64 z
```

#### command/rate_throttle (tobas_command_msgs/RateThrottle)

Commands the target angular velocity [rad/s] in the body frame and the throttle.
Specify `throttle` in the range 0 to 1.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector rate  # Target angular velocity expressed in the local frame [rad/s]
	float64 x
	float64 y
	float64 z
float64 throttle            # Target throttle [0, 1]
```

#### command/rate_throttle_vector (tobas_command_msgs/RateThrottleVector)

Commands the target angular velocity [rad/s] in the body frame, throttle, and thrust direction.
Specify `throttle` in the range 0 to 1 and `thrust_angle` as the thrust direction angle [rad].

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector rate  # Target angular velocity expressed in the local frame [rad/s]
	float64 x
	float64 y
	float64 z
float64 throttle            # Target throttle [0, 1]
float64 thrust_angle        # Target thrust angle [rad]
```

#### command/angle (tobas_command_msgs/Angle)

Commands the target attitude relative to the global frame using roll, pitch, and yaw Euler angles [rad].

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Euler angle  # Target euler angles expressed in the global frame [rad]
	float64 roll   # [rad]
	float64 pitch  # [rad]
	float64 yaw    # [rad]
```

#### command/angle_throttle (tobas_command_msgs/AngleThrottle)

Commands the target attitude [rad] relative to the global frame and the throttle.
Specify roll, pitch, and yaw in `angle` and a value from 0 to 1 in `throttle`.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Euler angle  # Target euler angles expressed in the global frame [rad]
	float64 roll   # [rad]
	float64 pitch  # [rad]
	float64 yaw    # [rad]
float64 throttle            # Target throttle [0, 1]
```

#### command/angle_throttle_vector (tobas_command_msgs/AngleThrottleVector)

Commands the target attitude [rad] relative to the global frame, throttle, and thrust direction.
Specify roll, pitch, and yaw in `angle`, a value from 0 to 1 in `throttle`, and the thrust direction angle [rad] in `thrust_angle`.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Euler angle  # Target euler angles expressed in the global frame [rad]
	float64 roll   # [rad]
	float64 pitch  # [rad]
	float64 yaw    # [rad]
float64 throttle            # Target throttle [0, 1]
float64 thrust_angle        # Target thrust angle [rad]
```

#### command/accel (tobas_command_msgs/Accel)

Commands the target linear acceleration in the global frame in m/s^2.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector accel  # Target linear acceleration expressed in the global frame [m/s^2]
	float64 x
	float64 y
	float64 z
```

#### command/accel_yaw (tobas_command_msgs/AccelYaw)

Commands the target linear acceleration [m/s^2] and yaw angle [rad] in the global frame.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector accel  # Target linear acceleration expressed in the global frame [m/s^2]
	float64 x
	float64 y
	float64 z
float64 yaw                  # Target yaw angle [rad]
```

#### command/accel_pitch_yaw (tobas_command_msgs/AccelPitchYaw)

Commands the target linear acceleration [m/s^2] and pitch and yaw angles [rad] in the global frame.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector accel  # Target linear acceleration expressed in the global frame [m/s^2]
	float64 x
	float64 y
	float64 z
float64 pitch                # Target pitch angle [rad]
float64 yaw                  # Target yaw angle [rad]
```

#### command/pos_vel_acc (tobas_command_msgs/PosVelAcc)

Commands the target position, velocity, and acceleration in the global frame together.
`pos` specifies position [m], `vel` velocity [m/s], and `acc` acceleration [m/s^2].

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector pos  # Target position expressed in the global frame [m]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector vel  # Target linear velocity expressed in the global frame [m/s]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector acc  # Target linear acceleration expressed in the global frame [m/s]
	float64 x
	float64 y
	float64 z
```

#### command/pos_vel_acc_yaw (tobas_command_msgs/PosVelAccYaw)

Commands the target position [m], velocity [m/s], acceleration [m/s^2], and yaw angle [rad] in the global frame.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector pos  # Target position expressed in the global frame [m]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector vel  # Target linear velocity expressed in the global frame [m/s]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector acc  # Target linear acceleration expressed in the global frame [m/s]
	float64 x
	float64 y
	float64 z
float64 yaw                # Target yaw angle [rad]
```

#### command/pos_vel_acc_pitch_yaw (tobas_command_msgs/PosVelAccPitchYaw)

Commands the target position [m], velocity [m/s], acceleration [m/s^2], and pitch and yaw angles [rad] in the global frame.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector pos  # Target position expressed in the global frame [m]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector vel  # Target linear velocity expressed in the global frame [m/s]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector acc  # Target linear acceleration expressed in the global frame [m/s]
	float64 x
	float64 y
	float64 z
float64 pitch              # Target pitch angle [rad]
float64 yaw                # Target yaw angle [rad]
```

#### command/speed_roll_delta_pitch (tobas_command_msgs/SpeedRollDeltaPitch)

Commands the target speed [m/s], roll angle [rad], and pitch angle offset from the trim attitude [rad] for a fixed-wing aircraft.
`delta_pitch` specifies the change relative to the trim pitch angle.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

float64 speed        # [m/s]
float64 roll         # [rad]
float64 delta_pitch  # [rad]
```

#### command/joint_positions (tobas_msgs/JointCommandArray)

Commands the target position for each joint.
In each element of `commands`, specify the joint name in `name` and the setpoint in `data`. Use angles [rad] for revolute joints and displacements [m] for prismatic joints.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_msgs/JointCommand[] commands
	string name
	float64 data
```

#### command/joint_velocities (tobas_msgs/JointCommandArray)

Commands the target velocity for each joint.
In each element of `commands`, specify the joint name in `name` and the setpoint in `data`. Use angular velocities [rad/s] for revolute joints and linear velocities [m/s] for prismatic joints.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_msgs/JointCommand[] commands
	string name
	float64 data
```

#### command/joint_efforts (tobas_msgs/JointCommandArray)

Commands the force or torque to apply to each joint.
In each element of `commands`, specify the joint name in `name` and the setpoint in `data`. Use torques [N·m] for revolute joints and forces [N] for prismatic joints.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_msgs/JointCommand[] commands
	string name
	float64 data
```

### Gazebo

These topics are used only in Gazebo simulations.

#### gazebo/ground_truth/battery (tobas_msgs/Battery)

Publishes the ground-truth voltage [V] and current [A] calculated by the Gazebo battery model.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 voltage  # [V]
float64 current  # [A]
```

#### gazebo/ground_truth/odom (tobas_msgs/OdometryWithCovarianceStamped)

Publishes the ground-truth position, orientation, linear and angular velocities, and linear and angular accelerations of the aircraft in Gazebo.
Position and orientation are expressed in the Gazebo world frame; velocities and accelerations are expressed in the body frame. All covariances are zero.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_msgs/OdometryWithCovariance odom
	tobas_msgs/Odometry odom
		tobas_kdl_msgs/Frame frame  # The transformation from the global frame to the body frame
			tobas_kdl_msgs/Vector trans
				float64 x
				float64 y
				float64 z
			tobas_kdl_msgs/Rotation rot
				float64[9] data
		tobas_kdl_msgs/Twist twist  # The 6D twist expressed in the body frame
			tobas_kdl_msgs/Vector linear
				float64 x
				float64 y
				float64 z
			tobas_kdl_msgs/Vector angular
				float64 x
				float64 y
				float64 z
		tobas_kdl_msgs/Accel accel  # The 6D accel expressed in the body frame
			tobas_kdl_msgs/Vector linear
				float64 x
				float64 y
				float64 z
			tobas_kdl_msgs/Vector angular
				float64 x
				float64 y
				float64 z
	tobas_eigen_msgs/Matrix3d position_covariance     # [m^2]
		float64[9] data
	tobas_eigen_msgs/Matrix3d orientation_covariance  # [rad^2]
		float64[9] data
	tobas_eigen_msgs/Matrix3d velocity_covariance     # [m^2/s^2]
		float64[9] data
	tobas_eigen_msgs/Matrix3d gyro_covariance         # [rad^2/s^2]
		float64[9] data
```

#### gazebo/ground_truth/wind (tobas_msgs/Wind)

Publishes the ground-truth wind velocity vector generated in Gazebo.
`vel` represents the velocity [m/s] along each axis of the global frame.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_kdl_msgs/Vector vel  # [m/s]
	float64 x
	float64 y
	float64 z
```

## Services

---

### Common

These services are available on both real hardware and in simulations.

#### set_arm (tobas_msgs/SetArm)

Changes the arming state of all rotors.
Set `arming` to `true` to request arming or `false` to request disarming.
Check `success` for success or failure and `message` for result details.

```txt
bool arming
---
bool success
string message
```

#### attach_load (tobas_msgs/AttachLoad)

Adds a load to the specified parent link in the aircraft's kinematic tree using a fixed joint.
In `load_id`, specify a non-empty identifier that is not already used by an attached load. In `parent_link`, specify an existing link name.
In `inertia`, specify the mass [kg], center of mass position relative to the parent link origin [m], and inertia tensor about the center of mass [kg·m^2]. Express the center of mass position and inertia tensor in the parent link frame.
On success, publishes the updated tree to the `kdl_tree` topic. On failure, `success` is set to `false`, and `message` contains the reason.

```txt
# Attach a load to the robot's kinematic tree with a fixed joint.

# Non-empty identifier that must be unique among currently attached loads.
string load_id

# Name of the existing link to which the load is attached.
string parent_link

# Load inertia expressed in the parent link frame:
# mass [kg], center of gravity relative to the parent link origin [m],
# and rotational inertia about the center of gravity [kg * m^2].
tobas_kdl_msgs/RigidBodyInertia inertia
	float64 mass                            # [kg]
	tobas_kdl_msgs/Vector cog               # [m]
		float64 x
		float64 y
		float64 z
	tobas_kdl_msgs/RotationalInertia i_cog  # [kg * m^2]
		float64[9] data  # [kg * m^2]
---
# True if the load was attached successfully.
bool success

# Error description on failure; empty on success.
string message
```

#### detach_load (tobas_msgs/DetachLoad)

Removes a load added with `attach_load` from the aircraft's kinematic tree by specifying `load_id`.
Use the same identifier as when attaching the load. Fails if the specified load is not attached.
On success, publishes the updated tree to the `kdl_tree` topic. On failure, `success` is set to `false`, and `message` contains the reason.

```txt
# Detach a previously attached load from the robot's kinematic tree.

# Identifier specified in the corresponding AttachLoad request.
string load_id
---
# True if the load was detached successfully.
bool success

# Error description on failure; empty on success.
string message
```

### Gazebo

These services are used only in Gazebo simulations.

#### gazebo/charge_battery (std_srvs/Empty)

Restores the Gazebo battery to a fully charged state.
The request and response have no fields to set or retrieve.

```txt
---
```

#### gazebo/lose_gnss_fix (std_srvs/Trigger)

Causes the Gazebo GNSS to lose its position fix.
After this call, `fix_type` in GNSS messages remains `NO_FIX` until the simulation is restarted.

```txt
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

#### gazebo/break_rotor/${rotor_link_name} (std_srvs/Trigger)

Simulates a failure of the specified rotor in Gazebo and sets the motor throttle command to zero.
Replace `${rotor_link_name}` with the target rotor's link name when calling this service.

```txt
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

#### gazebo/get_wind_parameters (tobas_gazebo_msgs/GetWindParams)

Retrieves the current parameters of the Gazebo wind model.
Returns the mean wind speed, wind direction, and gust multiplier, duration, and interval in `params`.

```txt
---
tobas_gazebo_msgs/WindParams params
	float64 mean_speed         # [m/s]
	float64 direction          # [rad]
	float64 gust_speed_factor  # [-]
	float64 gust_duration      # [s]
	float64 gust_interval      # [s]
```

#### gazebo/set_wind_parameters (tobas_gazebo_msgs/SetWindParams)

Changes the mean wind speed, wind direction, and gust multiplier, duration, and interval of the Gazebo wind model.
Specify the settings in `params` in the request, and check the applied values in `params` in the response.

```txt
tobas_gazebo_msgs/WindParams params
	float64 mean_speed         # [m/s]
	float64 direction          # [rad]
	float64 gust_speed_factor  # [-]
	float64 gust_duration      # [s]
	float64 gust_interval      # [s]
---
bool success
tobas_gazebo_msgs/WindParams params
	float64 mean_speed         # [m/s]
	float64 direction          # [rad]
	float64 gust_speed_factor  # [-]
	float64 gust_duration      # [s]
	float64 gust_interval      # [s]
```

#### gazebo/get_tether_parameters (tobas_gazebo_msgs/GetTetherParams)

Retrieves the current tension [N] and maximum cable length [m] of the Gazebo tether station.

```txt
---
tobas_gazebo_msgs/TetherParams params
	float64 tension         # [N]
	float64 maximum_length  # [m]
```

#### gazebo/set_tether_parameters (tobas_gazebo_msgs/SetTetherParams)

Changes the tension [N] and maximum cable length [m] of the Gazebo tether station.
Specify a tension of 0 or greater and a maximum length greater than 0. Check the applied values in `params` in the response.

```txt
tobas_gazebo_msgs/TetherParams params
	float64 tension         # [N]
	float64 maximum_length  # [m]
---
bool success
tobas_gazebo_msgs/TetherParams params
	float64 tension         # [N]
	float64 maximum_length  # [m]
```

#### gazebo/attach_fixed_load (tobas_gazebo_msgs/AttachFixedLoad)

Creates a box-shaped load in Gazebo and fixes it to the aircraft's attachment link.
In `load_pose`, specify the load's center of mass position and orientation relative to the attachment link. In `load_size`, specify its dimensions [m] along each axis of the load frame, and in `load_mass`, specify its mass [kg].
All dimensions and the mass must be positive. Fails if a fixed load is already attached.

```txt
geometry_msgs/Pose load_pose     # Pose of the load's center of mass relative to the attachment link
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1
geometry_msgs/Vector3 load_size  # Box dimensions along the load frame's x, y, z axes [m]
	float64 x
	float64 y
	float64 z
float64 load_mass                # Mass of the load [kg]
---
bool success
string message
```

#### gazebo/detach_fixed_load (tobas_gazebo_msgs/DetachFixedLoad)

Removes the joint connecting the fixed load to the aircraft in Gazebo.
The detached load remains in the simulation. Fails if no fixed load is attached.

```txt
---
bool success
string message
```

#### gazebo/attach_suspended_load (tobas_gazebo_msgs/AttachSuspendedLoad)

Creates a box-shaped load in Gazebo and suspends it from the aircraft by a cable.
In `attachment_point`, specify the attachment position in the body frame. In `load_size` and `load_mass`, specify the load's dimensions and mass, respectively.
Also specify the cable length, Young's modulus, and cross-sectional area. All dimensions, the mass, and all cable parameters must be positive. Fails if a suspended load is already attached.

```txt
geometry_msgs/Vector3 attachment_point  # Attachment point on the aircraft wrt. the local frame [m]
	float64 x
	float64 y
	float64 z
geometry_msgs/Vector3 load_size         # Box dimensions along the load frame's x, y, z axes [m]
	float64 x
	float64 y
	float64 z
float64 load_mass                       # [kg]
float64 cable_length                    # [m]
float64 cable_young_modulus             # [Pa]
float64 cable_cross_sectional_area      # [m^2]
---
bool success
string message
```

#### gazebo/detach_suspended_load (tobas_gazebo_msgs/DetachSuspendedLoad)

Disconnects the suspended load from the aircraft in Gazebo and stops applying cable forces.
The detached load remains in the simulation.

```txt
---
bool success
string message
```

## Actions

---

### Common

These actions are available on both real hardware and in simulations.

#### execute_mission (tobas_mission_msgs/ExecuteMission)

Executes the specified mission items in order.
In the goal, specify the item sequence in `mission.items` and the execution priority in `priority`.
Check `current_command_index` in the feedback for the item currently being executed. The result returns the error code, error description, and index of the last command.
See `tobas_mission_items` for the types and parameters of each item.

```txt
# Goal
tobas_mission_msgs/Mission mission
	tobas_mission_msgs/MissionItem[] items
		uint8 type
		byte[] data
tobas_mission_msgs/Priority priority
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1

---

# Result
tobas_mission_msgs/ErrorCode error_code
	int8 data
	int8 NO_ERROR = 0
	int8 MISSION_SUPERSEDED = -1
	int8 MANUAL_OVERRIDE = -2
	int8 ACCEPTANCE_TIMEOUT = -3
	int8 OTHER_ERROR = -4
string error_message
uint32 last_command_index

---

# Feedback
uint32 current_command_index
```

<!-- TODO: ミッションコマンドの詳細 -->
