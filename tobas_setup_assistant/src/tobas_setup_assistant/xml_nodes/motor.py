from xml.etree import ElementTree as ET
from typing import Tuple


class MotorModel(ET.Element):
    def __init__(
        self,
        ns: str,
        motor_number: int,
        link_name: str,
        joint_name: str,
        direction: str,
        rot_speed_coefs: Tuple[float, float],
        motor_const: float,
        moment_const: float,
        rotor_drag_coef: float,
        time_const_up: float,
        time_const_down: float,
    ) -> None:
        assert direction in {"cw", "ccw"}, direction
        assert rot_speed_coefs[0] >= 0.0 and rot_speed_coefs[1] >= 0.0
        assert motor_const >= 0.0, motor_const
        assert moment_const >= 0.0, moment_const
        assert rotor_drag_coef >= 0.0, rotor_drag_coef
        assert time_const_up >= 0.0, time_const_up
        assert time_const_down >= 0.0, time_const_down

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_rotor_plugin.so"
        plugin.attrib["name"] = f"{ns}_{motor_number}_rotor_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "motorNumber").text = str(motor_number)
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "jointName").text = joint_name
        ET.SubElement(plugin, "turningDirection").text = direction
        ET.SubElement(plugin, "rotSpeedCoefficients").text = " ".join(
            map(str, rot_speed_coefs)
        )
        ET.SubElement(plugin, "motorConstant").text = str(motor_const)
        ET.SubElement(plugin, "momentConstant").text = str(moment_const)
        ET.SubElement(plugin, "rotorDragCoefficient").text = str(rotor_drag_coef)
        ET.SubElement(plugin, "timeConstantUp").text = str(time_const_up)
        ET.SubElement(plugin, "timeConstantDown").text = str(time_const_down)
