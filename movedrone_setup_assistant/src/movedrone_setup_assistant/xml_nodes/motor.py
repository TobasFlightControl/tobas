from xml.etree import ElementTree as ET


class MotorModel(ET.Element):

    SLOWDOWN_SIM = 10.

    def __init__(
        self,
        ns: str,
        motor_number: int,
        link_name: str,
        joint_name: str,
        direction: str,
        max_rot_vel: float,
        motor_const: float,
        moment_const: float,
        rotor_drag_coef: float,
        time_const_up: float,
        time_const_down: float,
    ) -> None:
        assert direction in {"cw", "ccw"}
        assert max_rot_vel > 0.
        assert motor_const > 0.
        assert moment_const > 0.
        assert rotor_drag_coef > 0.
        assert time_const_up > 0.
        assert time_const_down > 0.

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libdh_gazebo_rotor_plugin.so"
        plugin.attrib["name"] = f'{ns}_{motor_number}_rotor_plugin'

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "motorNumber").text = f'{motor_number}'
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "jointName").text = joint_name
        ET.SubElement(plugin, "turningDirection").text = direction
        ET.SubElement(plugin, "maxRotVelocity").text = f'{max_rot_vel}'
        ET.SubElement(plugin, "motorConstant").text = f'{motor_const}'
        ET.SubElement(plugin, "momentConstant").text = f'{moment_const}'
        ET.SubElement(plugin, "rotorDragCoefficient").text = f'{rotor_drag_coef}'
        ET.SubElement(plugin, "timeConstantUp").text = f'{time_const_up}'
        ET.SubElement(plugin, "timeConstantDown").text = f'{time_const_down}'
        ET.SubElement(plugin, "motorSpeedPubTopic").text = f'motor_speed/{motor_number}'
        ET.SubElement(plugin, "commandSubTopic").text = "command/motor_speed"
        ET.SubElement(plugin, "rotorVelocitySlowdownSim").text = f'{self.SLOWDOWN_SIM}'
