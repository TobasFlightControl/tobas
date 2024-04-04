from xml.etree import ElementTree as ET
from typing import Tuple, List

from ..setting_widgets.fixed_wing.control_surfaces import ControlSurface


class FixedWingModel(ET.Element):
    def __init__(
        self,
        ns: str,
        link_name: str,
        altitude_0: float,
        wing_surface: float,
        wing_span: float,
        mean_aerodynamic_chord: float,
        aerodynamic_center: Tuple[float, float, float],
        alpha_limit: Tuple[float, float],
        c_lift_0: float,
        c_lift_alpha: float,
        c_drag_0: float,
        c_drag_alpha: float,
        c_side_beta: float,
        c_roll_beta: float,
        c_roll_p: float,
        c_roll_r: float,
        c_pitch_0: float,
        c_pitch_alpha: float,
        c_pitch_abs_beta: float,
        c_pitch_alpha_rate: float,
        c_pitch_q: float,
        c_yaw_beta: float,
        c_yaw_p: float,
        c_yaw_r: float,
        control_surfaces: List[ControlSurface],
    ):
        assert wing_surface > 0.0
        assert wing_span > 0.0
        assert alpha_limit[0] < 0.0 < alpha_limit[1]

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_fixed_wing_plugin.so"
        plugin.attrib["name"] = f"{ns}_fixed_wing_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "altitudeZero").text = str(altitude_0)

        # Vehicle
        ET.SubElement(plugin, "wingSurface").text = str(wing_surface)
        ET.SubElement(plugin, "wingSpan").text = str(wing_span)
        ET.SubElement(plugin, "meanAerodynamicChord").text = str(mean_aerodynamic_chord)
        ET.SubElement(plugin, "aerodynamicCenter").text = " ".join(map(str, aerodynamic_center))
        ET.SubElement(plugin, "lowerStallAngle").text = str(alpha_limit[0])
        ET.SubElement(plugin, "upperStallAngle").text = str(alpha_limit[1])

        # Aerodynamic Coefficients
        ET.SubElement(plugin, "cLift0").text = str(c_lift_0)
        ET.SubElement(plugin, "cLiftAlpha").text = str(c_lift_alpha)
        ET.SubElement(plugin, "cDrag0").text = str(c_drag_0)
        ET.SubElement(plugin, "cDragAlpha").text = str(c_drag_alpha)
        ET.SubElement(plugin, "cSideBeta").text = str(c_side_beta)
        ET.SubElement(plugin, "cRollBeta").text = str(c_roll_beta)
        ET.SubElement(plugin, "cRollP").text = str(c_roll_p)
        ET.SubElement(plugin, "cRollR").text = str(c_roll_r)
        ET.SubElement(plugin, "cPitch0").text = str(c_pitch_0)
        ET.SubElement(plugin, "cPitchAlpha").text = str(c_pitch_alpha)
        ET.SubElement(plugin, "cPitchAbsBeta").text = str(c_pitch_abs_beta)
        ET.SubElement(plugin, "cPitchAlphaRate").text = str(c_pitch_alpha_rate)
        ET.SubElement(plugin, "cPitchQ").text = str(c_pitch_q)
        ET.SubElement(plugin, "cYawBeta").text = str(c_yaw_beta)
        ET.SubElement(plugin, "cYawP").text = str(c_yaw_p)
        ET.SubElement(plugin, "cYawR").text = str(c_yaw_r)

        # Control Surfaces
        for idx, cs in enumerate(control_surfaces):
            cs_elem = ET.SubElement(plugin, "controlSurface")
            ET.SubElement(cs_elem, "index").text = str(idx)
            ET.SubElement(cs_elem, "jointName").text = cs.joint_name
            ET.SubElement(cs_elem, "minAngle").text = str(cs.min_angle)
            ET.SubElement(cs_elem, "maxAngle").text = str(cs.max_angle)
            ET.SubElement(cs_elem, "maxAngleRate").text = str(cs.max_angle_rate)
            ET.SubElement(cs_elem, "cLiftDelta").text = str(cs.c_lift_delta)
            ET.SubElement(cs_elem, "cDragAbsDelta").text = str(cs.c_drag_abs_delta)
            ET.SubElement(cs_elem, "cSideDelta").text = str(cs.c_side_delta)
            ET.SubElement(cs_elem, "cRollDelta").text = str(cs.c_roll_delta)
            ET.SubElement(cs_elem, "cPitchDelta").text = str(cs.c_pitch_delta)
            ET.SubElement(cs_elem, "cYawDelta").text = str(cs.c_yaw_delta)
