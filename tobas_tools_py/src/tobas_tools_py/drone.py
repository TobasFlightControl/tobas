import rospy
from dataclasses import dataclass, field

from .constants import *
from .joint_config import *
from .rotor_config import *
from .fixed_wing_tools import *


@dataclass
class Drone:
    joint_map: JointConfigMap = field(default_factory=list)
    rotors: RotorConfigs = field(default_factory=list)
    fixed_wing: FixedWingConfig = FixedWingConfig()
    has_fixed_wing: bool = False
    is_loaded: bool = False

    def load_from_param(self) -> None:
        self._get_joint_configs()
        self._get_rotor_configs()

        self.has_fixed_wing = rospy.search_param("fixed_wing")
        if self.has_fixed_wing:
            self._get_fixed_wing_config()

        self.is_loaded = True

    def _get_joint_configs(self) -> None:
        num_joints = rospy.get_param("num_joints")
        for joint_idx in range(num_joints):
            self._get_joint_config(joint_idx)

    def _get_joint_config(self, joint_idx: int) -> None:
        prefix = f"joint_{joint_idx}"
        cfg = JointConfig()

        name = rospy.get_param(f"{prefix}/name")

        cfg.home_pos = rospy.get_param(f"{prefix}/home_position")
        cfg.min_pos = rospy.get_param(f"{prefix}/min_position")
        cfg.max_pos = rospy.get_param(f"{prefix}/max_position")
        assert cfg.min_pos <= cfg.home_pos <= cfg.max_pos

        cmd_type = rospy.get_param(f"{prefix}/command_type")
        if cmd_type == JointCommandType.POSITION.value:
            cfg.cmd_type = JointCommandType.POSITION
        elif cmd_type == JointCommandType.VELOCITY.value:
            cfg.cmd_type = JointCommandType.VELOCITY
        elif cmd_type == JointCommandType.EFFORT.value:
            cfg.cmd_type = JointCommandType.EFFORT
        else:
            raise RuntimeError(f"Invalid command type: {cmd_type}")

        self.joint_map[name] = cfg

        return cfg

    def _get_rotor_configs(self) -> None:
        num_rotors = rospy.get_param("num_rotors")
        for rotor_idx in range(num_rotors):
            self.rotors.append(self._get_rotor_config(rotor_idx))

    def _get_rotor_config(self, rotor_idx: int) -> RotorConfig:
        prefix = f"rotor_{rotor_idx}"
        res = RotorConfig()

        # Link name
        res.link_name = rospy.get_param(f"{prefix}/link_name")

        # Direction
        direction = rospy.get_param(f"{prefix}/direction").lower()
        if direction == "ccw":
            res.direction = 1
        elif direction == "cw":
            res.direction = -1
        else:
            raise RuntimeError(
                f"Invalid rotation direction: {direction}. direction must be 'cw' or 'ccw'."
            )

        # Axis
        axis = rospy.get_param(f"{prefix}/axis").lower()
        if axis == Axis.X_POSITIVE.value:
            res.axis = Axis.X_POSITIVE
        elif axis == Axis.Z_POSITIVE.value:
            res.axis = Axis.Z_POSITIVE
        else:
            res.axis = Axis.UNKNOWN

        # ESC signal mode
        esc_signal_mode = rospy.get_param(f"{prefix}/esc_signal_mode").lower()
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
        res.num_poles = rospy.get_param(f"{prefix}/num_poles")
        assert res.num_poles > 0
        assert res.num_poles % 2 == 0

        res.max_rot_speed = rospy.get_param(f"{prefix}/max_rot_speed")
        res.motor_constant = rospy.get_param(f"{prefix}/motor_constant")
        res.moment_constant = rospy.get_param(f"{prefix}/moment_constant")
        res.drag_constant = rospy.get_param(f"{prefix}/drag_constant")

        res.rot_speed_coefs = rospy.get_param(f"{prefix}/rot_speed_coefs")
        assert len(res.rot_speed_coefs) == 2
        assert res.rot_speed_coefs[0] > 0.0
        assert res.rot_speed_coefs[1] >= 0.0

        res.pin = rospy.get_param(f"{prefix}/pin")
        assert MIN_PIN_ID <= res.pin <= MAX_PIN_ID

        return res

    def _get_fixed_wing_config(self) -> None:
        raise NotImplementedError()  # TODO
