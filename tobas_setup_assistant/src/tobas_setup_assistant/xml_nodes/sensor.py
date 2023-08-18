from xml.etree import ElementTree as ET
from typing import List


class SensorModel(ET.Element):

    def __init__(
        self,
        link_name: str,
        sensor_name: str,
        sensor_type: str,
        update_rate: float,
        always_on: bool = True,
        visualize: bool = False,
        pose: List[float] = [0.] * 6,
    ):
        assert update_rate >= 0.  # 0のときは寧ろ毎シミュレーション周期発行
        assert len(pose) == 6

        # robot/gazebo
        super().__init__("gazebo", reference=link_name)

        # robot/gazebo/self.sensor
        self.sensor = ET.SubElement(self, "sensor")
        self.sensor.attrib["name"] = sensor_name
        self.sensor.attrib["type"] = sensor_type

        ET.SubElement(self.sensor, "update_rate").text = str(update_rate)
        ET.SubElement(self.sensor, "always_on").text = str(always_on).lower()
        ET.SubElement(self.sensor, "visualize").text = str(visualize).lower()
        ET.SubElement(self.sensor, "pose").text = " ".join(map(str, pose))
