import math
from xml.etree import ElementTree as ET

from tobas_urdf_tools_py.core import *
from tobas_urdf_tools_py.dummy import DummyLink
from tobas_urdf_tools_py.gazebo import Camera
from tobas_urdf_tools_py.noise import GaussianNoise


def add_rgb_camera_model(
    robot: ET.Element,
    ns: str,
    link_name: str,
    offset: Origin,
    frame_rate: float,
    width: int,
    height: int,
    near: float,
    far: float,
    fov: float,
    noise_mean: float,
    noise_stddev: float,
) -> None:
    # TODO: リンク名やジョイント名が被っている場合は名前を変更する
    rgb_camera_link = f"{ns}/rgb_camera_link"
    optical_link = f"{ns}/rgb_camera_optical_link"

    # robot/link
    robot.append(DummyLink(rgb_camera_link))
    robot.append(DummyLink(optical_link))

    # robot/joint
    robot.append(
        Joint(name=f"{ns}/rgb_camera_joint", type="fixed", parent=link_name, child=rgb_camera_link, origin=offset)
    )
    robot.append(
        Joint(
            name=f"{ns}/rgb_camera_optical_joint",
            type="fixed",
            parent=rgb_camera_link,
            child=optical_link,
            origin=Origin(0.0, 0.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2),
        )
    )

    # robot/gazebo
    gazebo = ET.SubElement(robot, "gazebo")
    gazebo.attrib["reference"] = rgb_camera_link

    # robot/gazebo/sensor
    sensor = ET.SubElement(gazebo, "sensor")
    sensor.attrib["name"] = f"{ns}_rgb_camera"
    sensor.attrib["type"] = "camera"

    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = str(frame_rate)

    # robot/gazebo/sensor/camera
    camera = Camera(
        width=width,
        height=height,
        near=near,
        far=far,
        fov=fov,
        format=Camera.RGB8,
        noise=GaussianNoise(mean=noise_mean, stddev=noise_stddev),
    )
    sensor.append(camera)

    # robot/gazebo/sensor/plugin
    plugin = ET.SubElement(sensor, "plugin")
    plugin.attrib["name"] = f"{ns}_rgb_camera"
    plugin.attrib["filename"] = "libgazebo_ros_camera.so"

    ET.SubElement(plugin, "cameraName").text = "rgb_camera"
    ET.SubElement(plugin, "imageTopicName").text = "image_raw"
    ET.SubElement(plugin, "cameraInfoTopicName").text = "camera_info"

    ET.SubElement(plugin, "hackBaseline").text = "0.0"
    ET.SubElement(plugin, "distortionK1").text = "0.0"
    ET.SubElement(plugin, "distortionK2").text = "0.0"
    ET.SubElement(plugin, "distortionK3").text = "0.0"
    ET.SubElement(plugin, "distortionT1").text = "0.0"
    ET.SubElement(plugin, "distortionT2").text = "0.0"

    ET.SubElement(plugin, "robotNamespace").text = ns
    ET.SubElement(plugin, "frameName").text = optical_link
