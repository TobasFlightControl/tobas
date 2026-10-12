# ROS API

Tobas が提供する主要なトピック，サービス，アクションを説明します．
利用できる API は機体の構成や起動するノードによって異なります．実行時に`ros2 topic list`，`ros2 service list`，`ros2 action list`で確認してください．
外部端末から FC と通信する場合は，`remote_interface`名前空間で公開されている API を使用します．

## トピック

<!-- tobas_constants/ros_interfaces.hppの内容 -->
<!-- tobas_gazebo_common/constants.hppの内容 -->

---

### Common

実機とシミュレーションの両方で使用できるトピックです．

#### battery (tobas_msgs/Battery)

バッテリーの電圧 [V] と電流 [A] を配信する．

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

エンジンの回転速度 [rad/s]，燃料残量 [L]，油温 [degC] を配信する．

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

FC の CPU の動作周波数 [Hz]，温度 [degC]，負荷を配信する．

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

RC レシーバから受信した S.BUS のチャンネル値と受信状態を配信する．
`frame_lost`はフレームの欠落，`failsafe`はレシーバのフェイルセーフ状態を表す．

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

RC 入力を操縦用の値に変換して配信する．
ロール，ピッチ，スロットル，ヨーの入力に加え，飛行モード，操縦の有効・無効，キルスイッチ，汎用スイッチの状態を含む．
`status`で入力の受信状態を確認できる．

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

フィルタリング前の IMU データを配信する．
`accel`は加速度 [m/s^2]，`gyro`は角速度 [rad/s]，`dgyro`は角加速度 [rad/s^2] を表す．

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

フィルタリング後の IMU データを配信する．
`imu_raw`と同じ形式で，加速度，角速度，角加速度を含む．

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

磁気センサから取得した 3 軸の磁場ベクトルを配信する．
`mag`の各成分は無次元量として扱う．

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

気圧センサから取得した大気圧を Pa 単位で配信する．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 pressure  # [Pa]
```

#### gnss (tobas_msgs/Gnss)

GNSS による位置・速度の測定値と測位状態を配信する．
位置は緯度・経度，WGS 84 楕円体からの高さ，平均海面からの高さで表し，対地速度は ENU 座標系で表す．
位置・速度の共分散，測位種別，測位に使用した衛星数も含む．

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

各ロータの回転速度，推力，エラー状態を配信する．
`states`の各要素は`link_name`でロータを識別する．

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

可動ジョイントごとの位置，速度，力またはトルクを配信する．
`states`の各要素は`name`でジョイントを識別する．

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

状態推定器が推定した機体の位置・姿勢，並進・角速度，並進・角加速度を配信する．
位置・姿勢はグローバル座標系，速度・加速度は機体座標系で表す．
位置，姿勢，速度，角速度の推定誤差の共分散も含む．

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

制御器が現在追従している位置・姿勢，速度，加速度の目標値を配信する．
制御対象でない成分には NaN が入る．例えば，姿勢制御モードでは位置・速度の目標値が NaN になる．

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

ロータのアーム状態を配信する．
`data`が`true`ならアーム状態，`false`ならディスアーム状態を表す．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
bool data
```

### Command

これらのトピックに指令を発行すると，機体やジョイントの目標値を指定できます．
利用できる指令は機体フレームの型と飛行モードによって異なります．実行時に ROS 2 の CLI で対象トピックと購読ノードを確認してください．
`priority`を持つ指令では，通常指令を`NORMAL`，防御的な指令を`DEFENSIVE`，手動指令を`MANUAL`で指定します．

#### command/rate (tobas_command_msgs/Rate)

機体座標系の 3 軸まわりの目標角速度を rad/s 単位で指令する．

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

機体座標系の目標角速度 [rad/s] とスロットルを指令する．
`throttle`は 0 から 1 の範囲で指定する．

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

機体座標系の目標角速度 [rad/s]，スロットル，推力方向を指令する．
`throttle`は 0 から 1 の範囲，`thrust_angle`は推力方向の角度 [rad] を指定する．

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

グローバル座標系に対する目標姿勢を，ロール・ピッチ・ヨーのオイラー角 [rad] で指令する．

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

グローバル座標系に対する目標姿勢 [rad] とスロットルを指令する．
`angle`にロール・ピッチ・ヨーを，`throttle`に 0 から 1 の値を指定する．

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

グローバル座標系に対する目標姿勢 [rad]，スロットル，推力方向を指令する．
`angle`にロール・ピッチ・ヨー，`throttle`に 0 から 1 の値，`thrust_angle`に推力方向の角度 [rad] を指定する．

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

グローバル座標系の目標並進加速度を m/s^2 単位で指令する．

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

グローバル座標系の目標並進加速度 [m/s^2] と目標ヨー角 [rad] を指令する．

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

グローバル座標系の目標並進加速度 [m/s^2] と目標ピッチ角・ヨー角 [rad] を指令する．

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

グローバル座標系の目標位置，速度，加速度をまとめて指令する．
`pos`は位置 [m]，`vel`は速度 [m/s]，`acc`は加速度 [m/s^2] を指定する．

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

グローバル座標系の目標位置 [m]，速度 [m/s]，加速度 [m/s^2] と目標ヨー角 [rad] を指令する．

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

グローバル座標系の目標位置 [m]，速度 [m/s]，加速度 [m/s^2] と目標ピッチ角・ヨー角 [rad] を指令する．

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

固定翼機の目標速度 [m/s]，ロール角 [rad]，トリム姿勢からのピッチ角の差分 [rad] を指令する．
`delta_pitch`はトリム時のピッチ角を基準とした変化量を指定する．

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

ジョイントごとの目標位置を指令する．
`commands`の各要素にジョイント名`name`と目標値`data`を指定する．回転ジョイントでは角度 [rad]，直動ジョイントでは変位 [m] を用いる．

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

ジョイントごとの目標速度を指令する．
`commands`の各要素にジョイント名`name`と目標値`data`を指定する．回転ジョイントでは角速度 [rad/s]，直動ジョイントでは速度 [m/s] を用いる．

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

ジョイントごとに加える力またはトルクを指令する．
`commands`の各要素にジョイント名`name`と指令値`data`を指定する．回転ジョイントではトルク [N·m]，直動ジョイントでは力 [N] を用いる．

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

Gazebo シミュレーション時にのみ使用されるトピックです．

#### gazebo/ground_truth/battery (tobas_msgs/Battery)

Gazebo のバッテリーモデルが計算した電圧 [V] と電流 [A] の真値を配信する．

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

Gazebo 上の機体の位置・姿勢，並進・角速度，並進・角加速度の真値を配信する．
位置・姿勢は Gazebo のワールド座標系，速度・加速度は機体座標系で表す．共分散は全てゼロとなる．

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

Gazebo 上で生成された風速ベクトルの真値を配信する．
`vel`はグローバル座標系の各軸方向の速度 [m/s] を表す．

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

## サービス

---

### Common

実機とシミュレーションの両方で使用できるサービスです．

#### set_arm (tobas_msgs/SetArm)

全ロータのアーム状態を変更する．
`arming`に`true`を指定するとアーム，`false`を指定するとディスアームを要求する．
`success`で処理の成否，`message`で結果の詳細を確認できる．

```txt
bool arming
---
bool success
string message
```

#### attach_load (tobas_msgs/AttachLoad)

機体の運動学ツリーの指定した親リンクに，荷重を固定ジョイントで追加する．
`load_id`には取り付け済みの荷重と重複しない空でない識別子，`parent_link`には既存のリンク名を指定する．
`inertia`には質量 [kg]，親リンク原点に対する重心位置 [m]，重心まわりの慣性テンソル [kg·m^2] を指定する．重心位置と慣性テンソルは親リンク座標系で表す．
成功すると更新後のツリーを`kdl_tree`トピックに配信する．失敗時は`success`が`false`となり，`message`に理由が入る．

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

`attach_load`で追加した荷重を，`load_id`を指定して機体の運動学ツリーから削除する．
取り付け時と同じ識別子を使用する．指定した荷重が取り付けられていない場合は失敗する．
成功すると更新後のツリーを`kdl_tree`トピックに配信する．失敗時は`success`が`false`となり，`message`に理由が入る．

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

Gazebo シミュレーション時にのみ使用されるサービスです．

#### gazebo/charge_battery (std_srvs/Empty)

Gazebo のバッテリーを満充電の状態に戻す．
リクエストとレスポンスに指定・取得するフィールドはない．

```txt
---
```

#### gazebo/lose_gnss_fix (std_srvs/Trigger)

Gazebo の GNSS を測位不能の状態にする．
呼び出し後はシミュレーションを再起動するまで，GNSS メッセージの`fix_type`が`NO_FIX`になる．

```txt
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

#### gazebo/break_rotor/${rotor_link_name} (std_srvs/Trigger)

指定したロータを Gazebo 上で故障状態にし，モータへのスロットル指令をゼロにする．
`${rotor_link_name}`を対象ロータのリンク名に置き換えて呼び出す．

```txt
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

#### gazebo/get_wind_parameters (tobas_gazebo_msgs/GetWindParams)

Gazebo の風モデルに現在設定されているパラメータを取得する．
平均風速，風向，突風の倍率・継続時間・間隔を`params`に返す．

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

Gazebo の風モデルの平均風速，風向，突風の倍率・継続時間・間隔を変更する．
リクエストの`params`に設定値を指定し，レスポンスの`params`で反映された値を確認する．

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

Gazebo のテザーステーションに現在設定されている張力 [N] とケーブルの最大長 [m] を取得する．

```txt
---
tobas_gazebo_msgs/TetherParams params
	float64 tension         # [N]
	float64 maximum_length  # [m]
```

#### gazebo/set_tether_parameters (tobas_gazebo_msgs/SetTetherParams)

Gazebo のテザーステーションの張力 [N] とケーブルの最大長 [m] を変更する．
張力は 0 以上，最大長は 0 より大きい値を指定する．レスポンスの`params`で反映された値を確認できる．

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

Gazebo 上に直方体の荷物を生成し，機体の取り付け先リンクに固定する．
`load_pose`に取り付け先リンクに対する荷物の重心位置・姿勢，`load_size`に荷物座標系の各軸方向の寸法 [m]，`load_mass`に質量 [kg] を指定する．
各寸法と質量は正の値とする．既に固定荷物が取り付けられている場合は失敗する．

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

Gazebo 上で固定荷物と機体を結ぶジョイントを解除する．
取り外した荷物はシミュレーション内に残る．固定荷物が取り付けられていない場合は失敗する．

```txt
---
bool success
string message
```

#### gazebo/attach_suspended_load (tobas_gazebo_msgs/AttachSuspendedLoad)

Gazebo 上に直方体の荷物を生成し，ケーブルで機体から吊り下げる．
`attachment_point`に機体座標系での取り付け位置，`load_size`と`load_mass`に荷物の寸法・質量を指定する．
ケーブルの長さ，ヤング率，断面積も指定する．各寸法，質量，ケーブルの各パラメータは正の値とする．既に吊り下げ荷物が取り付けられている場合は失敗する．

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

Gazebo 上で吊り下げ荷物と機体の接続を解除し，ケーブルによる力の作用を停止する．
取り外した荷物はシミュレーション内に残る．

```txt
---
bool success
string message
```

## アクション

---

### Common

実機とシミュレーションの両方で使用できるアクションです．

#### execute_mission (tobas_mission_msgs/ExecuteMission)

指定したミッション項目を順番に実行する．
ゴールの`mission.items`に項目列，`priority`に実行優先度を指定する．
フィードバックの`current_command_index`で実行中の項目を確認できる．結果にはエラーコード，エラー内容，最後のコマンドのインデックスを返す．
各項目の種類とパラメータは`tobas_mission_items`を参照する．

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
