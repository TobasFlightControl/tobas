from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class VehicleParameters:
    wing_surface: float = 0.0  # Wing surface [m^2]
    wing_span: float = 0.0  # Wing span [m]
    mac: float = 0.0  # Mean Aerodynamic Chord [m]
    ac: Tuple[float, float, float] = (
        0.0,
        0.0,
        0.0,
    )  # Aerodynamic Center wrt the frame origin (NWU) [m]
    alpha_limit: Tuple[float, float] = (0.0, 0.0)  # Stall angles [rad]


@dataclass
class AerodynamicsCoefficients:
    # Lift force
    c_lift_0: float = 0.0  # [-]
    c_lift_alpha: float = 0.0  # [/rad]

    # Drag force
    c_drag_0: float = 0.0  # [-]
    c_drag_alpha: float = 0.0  # [/rad]

    # Side force
    c_side_beta: float = 0.0  # [/rad]

    # Roll moment
    c_roll_beta: float = 0.0  # [/rad]
    c_roll_p: float = 0.0  # [s/rad]
    c_roll_r: float = 0.0  # [s/rad]

    # Pitch moment
    c_pitch_0: float = 0.0  # [-]
    c_pitch_alpha: float = 0.0  # [/rad]
    c_pitch_abs_beta: float = 0.0  # [/rad]
    c_pitch_alpha_rate: float = 0.0  # [s/rad]
    c_pitch_q: float = 0.0  # [s/rad]

    # Yaw moment
    c_yaw_beta: float = 0.0  # [/rad]
    c_yaw_p: float = 0.0  # [s/rad]
    c_yaw_r: float = 0.0  # [s/rad]


@dataclass
class ControlSurface:
    index: int = 0  # 舵角配列における添字
    joint_name: str = ""
    angle_limit: Tuple[float, float] = (0.0, 0.0)
    max_angle_rate: float = 0.0

    c_lift_delta: float = 0.0  # [/rad]
    c_drag_abs_delta: float = 0.0  # [/rad], 舵角の正負にかかわらず抗力が発生するモデル
    c_side_delta: float = 0.0  # [/rad]
    c_roll_delta: float = 0.0  # [/rad]
    c_pitch_delta: float = 0.0  # [/rad]
    c_yaw_delta: float = 0.0  # [/rad]


ControlSurfaces = List[ControlSurface]


@dataclass
class FixedWingConfig:
    vehicle: VehicleParameters = VehicleParameters()
    aerodynamics: AerodynamicsCoefficients = AerodynamicsCoefficients()
    control_surfaces: ControlSurfaces = field(default_factory=list)

    def clear(self) -> None:
        self.control_surfaces.clear()
