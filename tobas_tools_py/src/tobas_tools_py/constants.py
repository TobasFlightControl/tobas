MIN_PWM = 1000
MAX_PWM = 2000
MIN_THROTTLE = 0.0
MAX_THROTTLE = 1.0

PROPERTY_SERVER_GCS = "/property_server_gcs"
PKG_EXTENSION = ".TBS"
CONTROLLER_NODE_NAME = "controller"
OBSERVER_NODE_NAME = "observer"


class RCRange:
    MIN = -1.0
    MAX = 1.0


class RCChannel:
    ROLL = 0  # CH1
    PITCH = 1  # CH2
    THROTTLE = 2  # CH3
    YAW = 3  # CH4
    MODE = 4  # CH5
    ESTOP = 6  # CH7
    GPSW = 7  # CH8


class Topic:
    TIME_REFERENCE = "/shm_driver/time_ref"
    BATTERY = "battery"
    BATTERY_LPF = "battery_filtered"
    CPU = "cpu"
    RC_INPUT = "rc_input"
    IMU = "imu"
    IMU_LPF = "imu_filtered"
    MAG = "magnetic_field"
    AIR_PRESSURE = "air_pressure"
    GNSS = "gps"
    LIDAR = "point_cloud"
    EXTERNAL_ODOM = "external_odometry"
    ROTOR_SPEEDS = "rotor_speeds"
    JOINT_STATES = "joint_states"
    ODOMETRY = "odom"
    EULER = "euler"
    WIND = "wind"
    EVENT = "event"
    MESSAGE = "message"
    LATENCY = "latency"
    ARMING = "arming"
    PRE_ARM_CHECK = "pre_arm_check"
    THRUST_CORRECTION_FACTOR = "thrust_correction_factor"

    class Command:
        THROTTLES = "command/throttles"
        ROTOR_SPEEDS = "command/rotor_speeds"
        DEFLECTION = "command/deflections"
        POS_VEL_ACC_YAW = "command/pos_vel_acc_yaw"
        POSITION_YAW = "command/position_yaw"
        VELOCITY_YAW = "command/velocity_yaw"
        RPY_THRUST = "command/rpy_thrust"
        POSE_TWIST_ACCEL = "command/pose_twist_accel"
        SPEED_ROLL_DPITCH = "command/speed_roll_delta_pitch"
        JOINT_POSITIONS = "command/joint_positions"
        JOINT_VELOCITIES = "command/joint_velocities"
        JOINT_EFFORTS = "command/joint_efforts"

    class Manipulation:
        POS_CTRL_JS = "joint_position_controller/target_joint_states"
        POS_CTRL_LS = "joint_position_controller/target_link_states"
        VEL_CTRL_JS = "joint_velocity_controller/target_joint_states"
        VEL_CTRL_LS = "joint_velocity_controller/target_link_states"
        EFF_CTRL_JS = "joint_effort_controller/target_joint_states"
        EFF_CTRL_LS = "joint_effort_controller/target_link_states"

    class Feedback:
        CONTROLLER = "feedback/controller"
        OBSERVER = "feedback/observer"

    class Throttled:
        BATTERY_LPF = "throttled/battery_filtered"
        RC_INPUT = "throttled/rc_input"
        EULER = "throttled/euler"
        LATENCY = "throttled/latency"


class Service:
    LIST_CONTROLLERS = "controller_manager/list_controllers"
    ENABLE_RC_OUTPUT = "enable_rc_output"
    GET_ARM = "get_arm"
    SET_ARM = "set_arm"
    GET_GNSS_ORIGIN = "get_gnss_origin"
    SET_GNSS_ORIGIN = "set_gnss_origin"
    PRE_ARM_CHECK = "pre_arm_check"
    RELOAD_CONFIG = "reload_config"
    START_MAIN_TIMER = "start_main_timer"
    STOP_MAIN_TIMER = "stop_main_timer"


class Action:
    TAKEOFF = "takeoff_action"
    LAND = "land_action"
    MOVE = "move_action"
