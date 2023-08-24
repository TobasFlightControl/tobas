from xml.etree import ElementTree as ET
from typing import Tuple

from urdf_tools_py.core import *
from urdf_tools_py.dummy import DummyLink


def add_lidar_model(
    robot: ET.Element,
    ns: str,
    link_name: str,
    offset: Origin,
    update_rate: float,
    hor_samples: int,
    ver_samples: int,
    hor_fov: Tuple[float, float],
    ver_fov: Tuple[float, float],
    dist_range: Tuple[float, float],
    resolution: float,
    noise_stddev: float,
) -> None:
    assert update_rate > 0.
    assert hor_samples > 0
    assert ver_samples > 0
    assert hor_fov[0] <= hor_fov[1]
    assert ver_fov[0] <= ver_fov[1]
    assert dist_range[0] <= dist_range[1]
    assert resolution > 0.
    assert noise_stddev >= 0.

    # TODO: リンク名やジョイント名が被っている場合は名前を変更する
    lidar_link = f'{ns}/lidar_link'

    # robot/link
    robot.append(DummyLink(lidar_link))

    # robot/joint
    robot.append(Joint(
        name=f'{ns}/lidar_joint',
        type="fixed",
        parent=link_name,
        child=lidar_link,
        origin=offset,
    ))

    # robot/gazebo
    gazebo = ET.SubElement(robot, "gazebo")
    gazebo.attrib["reference"] = lidar_link

    # robot/gazebo/sensor
    sensor = ET.SubElement(gazebo, "sensor")
    sensor.attrib["name"] = f'{ns}_lidar'
    sensor.attrib["type"] = "ray"
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = str(update_rate)
    ET.SubElement(sensor, "visualize").text = "false"

    # robot/gazebo/sensor/ray
    ray = ET.SubElement(sensor, "ray")

    # robot/gazebo/sensor/ray/scan
    scan = ET.SubElement(ray, "scan")

    # robot/gazebo/sensor/ray/scan/horizontal
    horizontal = ET.SubElement(scan, "horizontal")
    ET.SubElement(horizontal, "samples").text = str(hor_samples)
    ET.SubElement(horizontal, "resolution").text = "1"
    ET.SubElement(horizontal, "min_angle").text = str(hor_fov[0])
    ET.SubElement(horizontal, "max_angle").text = str(hor_fov[1])

    # robot/gazebo/sensor/ray/scan/vertical
    vertical = ET.SubElement(scan, "vertical")
    ET.SubElement(vertical, "samples").text = str(ver_samples)
    ET.SubElement(vertical, "resolution").text = "1"
    ET.SubElement(vertical, "min_angle").text = str(ver_fov[0])
    ET.SubElement(vertical, "max_angle").text = str(ver_fov[1])

    # robot/gazebo/sensor/ray/range
    range = ET.SubElement(ray, "range")
    ET.SubElement(range, "min").text = str(dist_range[0])
    ET.SubElement(range, "max").text = str(dist_range[1])
    ET.SubElement(range, "resolution").text = str(resolution)

    # robot/gazebo/sensor/plugin
    plugin = ET.SubElement(sensor, "plugin")
    plugin.attrib["filename"] = "libtobas_gazebo_lidar_plugin.so"
    plugin.attrib["name"] = "tobas_gazebo_lidar_plugin"
    ET.SubElement(plugin, "robotNamespace").text = ns
    ET.SubElement(plugin, "frameName").text = link_name
    ET.SubElement(plugin, "topicName").text = "lidar"
    ET.SubElement(plugin, "gaussianNoiseStddev").text = str(noise_stddev)
