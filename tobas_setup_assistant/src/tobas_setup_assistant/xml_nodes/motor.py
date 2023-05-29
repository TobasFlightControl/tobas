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
        assert direction in {"cw", "ccw"}, direction
        assert max_rot_vel > 0., max_rot_vel
        assert motor_const > 0., motor_const
        assert moment_const > 0., moment_const
        assert rotor_drag_coef > 0., rotor_drag_coef
        assert time_const_up > 0., time_const_up
        assert time_const_down > 0., time_const_down

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_rotor_plugin.so"
        plugin.attrib["name"] = f'{ns}_{motor_number}_rotor_plugin'

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "motorNumber").text = str(motor_number)
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "jointName").text = joint_name
        ET.SubElement(plugin, "turningDirection").text = direction
        ET.SubElement(plugin, "maxRotVelocity").text = str(max_rot_vel)
        ET.SubElement(plugin, "motorConstant").text = str(motor_const)
        ET.SubElement(plugin, "momentConstant").text = str(moment_const)
        ET.SubElement(plugin, "rotorDragCoefficient").text = str(rotor_drag_coef)
        ET.SubElement(plugin, "timeConstantUp").text = str(time_const_up)
        ET.SubElement(plugin, "timeConstantDown").text = str(time_const_down)
        ET.SubElement(plugin, "debugPubTopic").text = f'ground_truth/rotor_debug/{motor_number}'
        ET.SubElement(plugin, "commandSubTopic").text = "command/motor_speed"
        ET.SubElement(plugin, "rotorSpeedSlowdownSim").text = str(self.SLOWDOWN_SIM)
