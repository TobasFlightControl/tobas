# ROS API

This does not cover all APIs; only the main ones are described here.
For details, check at runtime with `$ ros2 topic list` and similar tools.
You can also communicate with the FC from an external terminal by using the APIs in the `remote_interface` namespace.

## Topics

<!-- tobas_constants/ros_interfaces.hppの内容 -->
<!-- tobas_gazebo_common/constants.hppの内容 -->

---

### Common

These topics can be used on both real hardware and in simulation.

#### battery (tobas_msgs/Battery)

Battery status.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 voltage  # [V]
float64 current  # [A]
```

### engine_state (tobas_msgs/EngineState)

Engine status.

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

CPU status.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 temperature  # [degC]
uint64 frequency     # [Hz]
float64 load         # [-]
```

#### sbus (tobas_msgs/Sbus)

Raw S.BUS messages obtained from the RC receiver.

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

A converted S.BUS message in a more convenient format.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
bool ok

float64 roll      # CH1: [-1, 1]
float64 pitch     # CH2: [-1, 1]
float64 throttle  # CH3: [-1, 1]
float64 yaw       # CH4: [-1, 1]
uint8 mode        # CH7: Flight Mode
bool sub_mode     # CH8: Sub Flight Mode
bool enable       # CH5: Enable Radio Control
bool kill         # CH6: Kill Switch
bool[8] gpsw      # CH9-16: General Purpose Switch
```

#### imu_raw (tobas_msgs/Imu)

IMU data before filtering.

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

IMU data after filtering.

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

3-axis geomagnetic field.

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

Atmospheric pressure.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 pressure  # [Pa]
```

#### gnss (tobas_msgs/Gnss)

Position and velocity obtained from GNSS.

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
float64 latitude                                # [deg]
float64 longitude                               # [deg]
float64 altitude                                # [m]
tobas_eigen_msgs/Matrix3d position_covariance   # [m^2]
	float64[9] data

# Velocity
tobas_kdl_msgs/Vector ground_speed                 # [m/s]
	float64 x
	float64 y
	float64 z
tobas_eigen_msgs/Matrix3d velocity_covariance      # [m^2/s^2]
	float64[9] data
```

#### rotor_states (tobas_msgs/RotorStateArray)

State of each rotor.

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

State of movable joints.

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

Position, velocity, and acceleration relative to the startup position, as estimated by the state estimator.

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

Current setpoint of the controller.
Uncontrolled values (for example, position and velocity in attitude control mode) are set to NaN.

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

Whether all rotors are armed.

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
bool data
```

### Command

Users can control the drone by publishing these topics from inside the FC.
Accepted commands depend on the airframe type and flight mode, so check with the ROS 2 CLI.

#### command/rate (tobas_command_msgs/Rate)

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

Position commands for joints.

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

Velocity commands for joints.

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

Force commands for joints.

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

These topics are used only during Gazebo simulation.

#### gazebo/ground_truth/battery (tobas_msgs/Battery)

Ground-truth battery status.

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

Ground-truth position, velocity, and acceleration relative to the startup position.

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

Ground-truth wind velocity in the global coordinate system.

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

These services can be used on both real hardware and in simulation.

#### set_arm (tobas_msgs/SetArm)

Change the arm state of all rotors.

```txt
bool arming
---
bool success
string message
```

### Gazebo

These services are used only during Gazebo simulation.

#### gazebo/charge_battery (std_srvs/Empty)

Fully charge the battery.

```txt
---
```

#### gazebo/get_wind_parameters (tobas_gazebo_msgs/GetWindParams)

Get the parameters used to generate wind in the simulation.

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

Set the parameters used to generate wind in the simulation.

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

Get parameters related to the tether station.

```txt
---
tobas_gazebo_msgs/TetherParams params
	float64 tension         # [N]
	float64 maximum_length  # [m]
```

#### gazebo/set_tether_parameters (tobas_gazebo_msgs/SetTetherParams)

Set parameters related to the tether station.

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

#### gazebo/attach_suspended_load (tobas_gazebo_msgs/AttachSuspendedLoad)

Attach a suspended load.

```txt
geometry_msgs/Vector3 attachment_point  # [m] Attachment point on the aircraft wrt. the local frame
	float64 x
	float64 y
	float64 z
float64 load_sx                         # [m]
float64 load_sy                         # [m]
float64 load_sz                         # [m]
float64 load_mass                       # [kg]
float64 cable_length                    # [m]
float64 cable_young_modulus             # [Pa]
float64 cable_cross_sectional_area      # [m^2]
---
bool success
string message
```

#### gazebo/detach_suspended_load (tobas_gazebo_msgs/DetachSuspendedLoad)

Detach a suspended load.

```txt
---
bool success
string message
```

#### gazebo/break_rotor/${rotor_link_name} (std_srvs/Trigger)

Force a motor to stop.

```txt
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

## Actions

---

### Common

These actions can be used on both real hardware and in simulation.

#### execute_mission (tobas_mission_msgs/ExecuteMission)

Execute a sequence of missions.
For details of each command, see `tobas_mission_items`.

```txt
# Goal
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

---

# Feedback
uint32 current_index
```

<!-- TODO: ミッションコマンドの詳細 -->
