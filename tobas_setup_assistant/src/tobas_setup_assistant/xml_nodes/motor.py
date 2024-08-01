from xml.etree import ElementTree as ET
from typing import Tuple

from tobas_tools_py.rotor_config import TurningDirection


class MotorModel(ET.Element):
    def __init__(
        self,
        ns: str,
        motor_number: int,
        link_name: str,
        joint_name: str,
        rot_speed_coefs: Tuple[float, float],
        motor_const: float,
        moment_const: float,
        rotor_drag_coef: float,
        direction: str,
        time_const_up: float,
        time_const_down: float,
        max_rot_speed: float,
        num_poles: int,
        max_current: float,
        esc_mode: str,
        max_model_error_rate: float,
    ) -> None:
        assert rot_speed_coefs[0] >= 0.0 and rot_speed_coefs[1] >= 0.0
        assert motor_const >= 0.0, motor_const
        assert moment_const >= 0.0, moment_const
        assert rotor_drag_coef >= 0.0, rotor_drag_coef
        assert direction in {
            TurningDirection.CW.name,
            TurningDirection.CCW.name,
        }, direction
        assert time_const_up > 0.0, time_const_up
        assert time_const_down > 0.0, time_const_down
        assert max_rot_speed > 0.0, max_rot_speed
        assert num_poles > 0 and num_poles % 2 == 0, num_poles
        assert max_current > 0.0, max_current
        assert max_model_error_rate >= 0.0, max_model_error_rate

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_rotor_plugin.so"
        plugin.attrib["name"] = f"{ns}_{motor_number}_rotor_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "motorNumber").text = str(motor_number)
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "jointName").text = joint_name
        ET.SubElement(plugin, "rotSpeedCoefficients").text = " ".join(map(str, rot_speed_coefs))
        ET.SubElement(plugin, "motorConstant").text = str(motor_const)
        ET.SubElement(plugin, "momentConstant").text = str(moment_const)
        ET.SubElement(plugin, "rotorDragCoefficient").text = str(rotor_drag_coef)
        ET.SubElement(plugin, "turningDirection").text = direction
        ET.SubElement(plugin, "timeConstantUp").text = str(time_const_up)
        ET.SubElement(plugin, "timeConstantDown").text = str(time_const_down)
        ET.SubElement(plugin, "maxRotationSpeed").text = str(max_rot_speed)
        ET.SubElement(plugin, "numPoles").text = str(num_poles)
        ET.SubElement(plugin, "maxCurrent").text = str(max_current)
        ET.SubElement(plugin, "escMode").text = esc_mode
        ET.SubElement(plugin, "maxModelErrorRate").text = str(max_model_error_rate)
