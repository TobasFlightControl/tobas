import rospy
import yaml
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from overrides import override
from typing import final

from .constants import *
from .joint_config import *
from .rotor_config import *
from .fixed_wing_tools import *


@dataclass
class Drone:
    drone_name: str = ""
    battery_nominal_voltage: float = 0.0
    joint_map: JointConfigMap = field(default_factory=dict)
    rotors: RotorConfigs = field(default_factory=list)
    fixed_wing: FixedWingConfig = FixedWingConfig()
    has_fixed_wing: bool = False
    is_loaded: bool = False


class DroneLoader(ABC):

    DRONE_NAME = "drone_name"
    BATTERY_NOMINAL_VOLTAGE = "battery_nominal_voltage"

    NUM_JOINTS = "num_joints"
    JOINT_PREFIX = "joint_"
    JOINT_NAME = "name"
    HOME_POSITION = "home_position"
    MIN_POSITION = "min_position"
    MAX_POSITION = "max_position"
    COMMAND_TYPE = "command_type"

    NUM_ROTORS = "num_rotors"
    ROTOR_PREFIX = "rotor_"
    LINK_NAME = "link_name"
    DIRECTION = "direction"
    AXIS = "axis"
    ESC_SIGNAL_MODE = "esc_signal_mode"
    NUM_POLES = "num_poles"
    MAX_ROT_SPEED = "max_rot_speed"
    MOTOR_CONSTANT = "motor_constant"
    MOMENT_CONSTANT = "moment_constant"
    DRAG_CONSTANT = "drag_constant"
    ROT_SPEED_COEFS = "rot_speed_coefs"
    CHANNEL = "channel"

    FIXED_WING = "fixed_wing"

    CCW = "ccw"
    CW = "cw"

    def __init__(self, drone: Drone) -> None:
        self._drone = drone

    @abstractmethod
    def load(self) -> None:
        raise NotImplementedError()

    @final
    def _clear(self) -> None:
        self._drone.joint_map.clear()
        self._drone.rotors.clear()
        self._drone.fixed_wing.clear()


class DroneLoader_Param(DroneLoader):
    @override
    def load(self) -> None:
        self._clear()

        self._drone.drone_name = rospy.get_param(self.DRONE_NAME)
        self._drone.battery_nominal_voltage = rospy.get_param(self.BATTERY_NOMINAL_VOLTAGE)

        self._get_joint_configs()
        self._get_rotor_configs()

        self._drone.has_fixed_wing = rospy.search_param(self.FIXED_WING)
        if self._drone.has_fixed_wing:
            self._get_fixed_wing_config()

        self._drone.is_loaded = True

    def _get_joint_configs(self) -> None:
        num_joints = rospy.get_param(self.NUM_JOINTS)
        for joint_idx in range(num_joints):
            self._get_joint_config(joint_idx)

    def _get_joint_config(self, joint_idx: int) -> None:
        prefix = f"{self.JOINT_PREFIX}{joint_idx}/"
        cfg = JointConfig()

        cfg.home_pos = rospy.get_param(f"{prefix}{self.HOME_POSITION}")
        cfg.min_pos = rospy.get_param(f"{prefix}{self.MIN_POSITION}")
        cfg.max_pos = rospy.get_param(f"{prefix}{self.MAX_POSITION}")
        assert cfg.min_pos <= cfg.home_pos <= cfg.max_pos

        cmd_type = rospy.get_param(f"{prefix}{self.COMMAND_TYPE}")
        if cmd_type == JointCommandType.POSITION.value:
            cfg.cmd_type = JointCommandType.POSITION
        elif cmd_type == JointCommandType.VELOCITY.value:
            cfg.cmd_type = JointCommandType.VELOCITY
        elif cmd_type == JointCommandType.EFFORT.value:
            cfg.cmd_type = JointCommandType.EFFORT
        else:
            raise RuntimeError(f"Invalid command type: {cmd_type}")

        name = rospy.get_param(f"{prefix}{self.JOINT_NAME}")
        self._drone.joint_map[name] = cfg

        return cfg

    def _get_rotor_configs(self) -> None:
        num_rotors = rospy.get_param(self.NUM_ROTORS)
        for rotor_idx in range(num_rotors):
            self._drone.rotors.append(self._get_rotor_config(rotor_idx))

    def _get_rotor_config(self, rotor_idx: int) -> RotorConfig:
        prefix = f"{self.ROTOR_PREFIX}{rotor_idx}/"
        res = RotorConfig()

        # Link name
        res.link_name = rospy.get_param(f"{prefix}{self.LINK_NAME}")

        # Direction
        direction = rospy.get_param(f"{prefix}{self.DIRECTION}").lower()
        if direction == self.CCW:
            res.direction = 1
        elif direction == self.CW:
            res.direction = -1
        else:
            raise RuntimeError(f'Invalid rotation direction: {direction}. It must be "{self.CW}" or "{self.CCW}".')

        # Axis
        axis = rospy.get_param(f"{prefix}{self.AXIS}").lower()
        if axis == Axis.X_POSITIVE.value:
            res.axis = Axis.X_POSITIVE
        elif axis == Axis.Z_POSITIVE.value:
            res.axis = Axis.Z_POSITIVE
        else:
            res.axis = Axis.UNKNOWN

        # ESC signal mode
        esc_signal_mode = rospy.get_param(f"{prefix}{self.ESC_SIGNAL_MODE}").lower()
        if esc_signal_mode == EscSignalMode.BLHELI_OPEN_LOOP.value:
            res.esc_signal_mode = EscSignalMode.BLHELI_OPEN_LOOP
        elif esc_signal_mode == EscSignalMode.BLHELI_CLOSED_LOOP_LOW_RANGE.value:
            res.esc_signal_mode = EscSignalMode.BLHELI_CLOSED_LOOP_LOW_RANGE
        elif esc_signal_mode == EscSignalMode.BLHELI_CLOSED_LOOP_MID_RANGE.value:
            res.esc_signal_mode = EscSignalMode.BLHELI_CLOSED_LOOP_MID_RANGE
        elif esc_signal_mode == EscSignalMode.BLHELI_CLOSED_LOOP_HIGH_RANGE.value:
            res.esc_signal_mode = EscSignalMode.BLHELI_CLOSED_LOOP_HIGH_RANGE
        else:
            raise RuntimeError(f"Invalid ESC signal mode: {esc_signal_mode}")

        # The number of poles
        res.num_poles = rospy.get_param(f"{prefix}{self.NUM_POLES}")
        assert res.num_poles > 0
        assert res.num_poles % 2 == 0

        res.max_rot_speed = rospy.get_param(f"{prefix}{self.MAX_ROT_SPEED}")
        res.motor_constant = rospy.get_param(f"{prefix}{self.MOTOR_CONSTANT}")
        res.moment_constant = rospy.get_param(f"{prefix}{self.MOMENT_CONSTANT}")
        res.drag_constant = rospy.get_param(f"{prefix}{self.DRAG_CONSTANT}")

        res.rot_speed_coefs = rospy.get_param(f"{prefix}{self.ROT_SPEED_COEFS}")
        assert len(res.rot_speed_coefs) == 2
        assert res.rot_speed_coefs[0] > 0.0
        assert res.rot_speed_coefs[1] >= 0.0

        res.channel = rospy.get_param(f"{prefix}{self.CHANNEL}")
        assert 0 <= res.channel < SERVO_RAIL_SIZE

        return res

    def _get_fixed_wing_config(self) -> None:
        raise NotImplementedError()  # TODO


class DroneLoader_File(DroneLoader):
    def __init__(self, drone: Drone, tbsf_path: str) -> None:
        super().__init__(drone)
        self._tbsf_path = tbsf_path
        self._data = dict()

    @override
    def load(self) -> None:
        assert self._tbsf_path.endswith(".tbsf")

        with open(self._tbsf_path, "r") as f:
            self._data = yaml.safe_load(f)

        self._clear()

        self._drone.drone_name = self._data[self.DRONE_NAME]
        self._drone.battery_nominal_voltage = self._data[self.BATTERY_NOMINAL_VOLTAGE]

        self._get_joint_configs()
        self._get_rotor_configs()

        self._drone.has_fixed_wing = self.FIXED_WING in self._data
        if self._drone.has_fixed_wing:
            self._get_fixed_wing_config()

        self._drone.is_loaded = True

    def _get_joint_configs(self) -> None:
        num_joints = self._data[self.NUM_JOINTS]
        for joint_idx in range(num_joints):
            self._get_joint_config(joint_idx)

    def _get_joint_config(self, joint_idx: int) -> None:
        joint_data = self._data[f"{self.JOINT_PREFIX}{joint_idx}"]
        cfg = JointConfig()

        cfg.home_pos = joint_data[self.HOME_POSITION]
        cfg.min_pos = joint_data[self.MIN_POSITION]
        cfg.max_pos = joint_data[self.MAX_POSITION]
        assert cfg.min_pos <= cfg.home_pos <= cfg.max_pos

        cmd_type = joint_data[self.COMMAND_TYPE]
        if cmd_type == JointCommandType.POSITION.value:
            cfg.cmd_type = JointCommandType.POSITION
        elif cmd_type == JointCommandType.VELOCITY.value:
            cfg.cmd_type = JointCommandType.VELOCITY
        elif cmd_type == JointCommandType.EFFORT.value:
            cfg.cmd_type = JointCommandType.EFFORT
        else:
            raise RuntimeError(f"Invalid command type: {cmd_type}")

        name = joint_data[self.JOINT_NAME]
        self._drone.joint_map[name] = cfg

        return cfg

    def _get_rotor_configs(self) -> None:
        num_rotors = self._data[self.NUM_ROTORS]
        for rotor_idx in range(num_rotors):
            self._drone.rotors.append(self._get_rotor_config(rotor_idx))

    def _get_rotor_config(self, rotor_idx: int) -> RotorConfig:
        rotor_data = self._data[f"{self.ROTOR_PREFIX}{rotor_idx}"]
        res = RotorConfig()

        # Link name
        res.link_name = rotor_data[self.LINK_NAME]

        # Direction
        direction = rotor_data[self.DIRECTION].lower()
        if direction == self.CCW:
            res.direction = 1
        elif direction == self.CW:
            res.direction = -1
        else:
            raise RuntimeError(f'Invalid rotation direction: {direction}. It must be "{self.CW}" or "{self.CCW}".')

        # Axis
        axis = rotor_data[self.AXIS].lower()
        if axis == Axis.X_POSITIVE.value:
            res.axis = Axis.X_POSITIVE
        elif axis == Axis.Z_POSITIVE.value:
            res.axis = Axis.Z_POSITIVE
        else:
            res.axis = Axis.UNKNOWN

        # ESC signal mode
        esc_signal_mode = rotor_data[self.ESC_SIGNAL_MODE].lower()
        if esc_signal_mode == EscSignalMode.BLHELI_OPEN_LOOP.value:
            res.esc_signal_mode = EscSignalMode.BLHELI_OPEN_LOOP
        elif esc_signal_mode == EscSignalMode.BLHELI_CLOSED_LOOP_LOW_RANGE.value:
            res.esc_signal_mode = EscSignalMode.BLHELI_CLOSED_LOOP_LOW_RANGE
        elif esc_signal_mode == EscSignalMode.BLHELI_CLOSED_LOOP_MID_RANGE.value:
            res.esc_signal_mode = EscSignalMode.BLHELI_CLOSED_LOOP_MID_RANGE
        elif esc_signal_mode == EscSignalMode.BLHELI_CLOSED_LOOP_HIGH_RANGE.value:
            res.esc_signal_mode = EscSignalMode.BLHELI_CLOSED_LOOP_HIGH_RANGE
        else:
            raise RuntimeError(f"Invalid ESC signal mode: {esc_signal_mode}")

        # The number of poles
        res.num_poles = rotor_data[self.NUM_POLES]
        assert res.num_poles > 0
        assert res.num_poles % 2 == 0

        res.max_rot_speed = rotor_data[self.MAX_ROT_SPEED]
        res.motor_constant = rotor_data[self.MOTOR_CONSTANT]
        res.moment_constant = rotor_data[self.MOMENT_CONSTANT]
        res.drag_constant = rotor_data[self.DRAG_CONSTANT]

        res.rot_speed_coefs = rotor_data[self.ROT_SPEED_COEFS]
        assert len(res.rot_speed_coefs) == 2
        assert res.rot_speed_coefs[0] > 0.0
        assert res.rot_speed_coefs[1] >= 0.0

        res.channel = rotor_data[self.CHANNEL]
        assert 0 <= res.channel < SERVO_RAIL_SIZE

        return res

    def _get_fixed_wing_config(self) -> None:
        raise NotImplementedError()  # TODO
