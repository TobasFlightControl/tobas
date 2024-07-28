import math
from xml.etree import ElementTree as ET

from tobas_urdf_tools_py.core import Joint, Origin
from tobas_urdf_tools_py.dummy import DummyInertialLink
from tobas_urdf_tools_py.gazebo import Camera


def add_depth_camera_model(
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
    baseline: float,
    noise_model: str,
) -> None:
    # TODO: リンク名やジョイント名が被っている場合は名前を変更する
    depth_camera_link = f"{ns}/depth_camera_link"
    optical_link = f"{ns}/depth_camera_optical_link"

    # robot/link
    robot.append(DummyInertialLink(depth_camera_link))
    robot.append(DummyInertialLink(optical_link))

    # robot/joint
    robot.append(
        Joint(name=f"{ns}/depth_camera_joint", type="fixed", parent=link_name, child=depth_camera_link, origin=offset)
    )
    robot.append(
        Joint(
            name=f"{ns}/depth_camera_optical_joint",
            type="fixed",
            parent=depth_camera_link,
            child=optical_link,
            origin=Origin(0.0, 0.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2),
        )
    )

    # robot/gazebo
    gazebo = ET.SubElement(robot, "gazebo")
    gazebo.attrib["reference"] = depth_camera_link

    # robot/gazebo/sensor
    sensor = ET.SubElement(gazebo, "sensor")
    sensor.attrib["name"] = f"{ns}_depth_camera"
    sensor.attrib["type"] = "depth"

    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = str(frame_rate)

    # robot/gazebo/sensor/camera
    camera = Camera(
        width=width,
        height=height,
        near=near,
        far=far,
        fov=fov,
        format=Camera.L8,
        noise=None,  # ノイズは別で加えるため，cameraタブではノイズなしにする
    )
    sensor.append(camera)

    # robot/gazebo/sensor/plugin
    plugin = ET.SubElement(sensor, "plugin")
    plugin.attrib["name"] = f"{ns}_depth_camera"
    plugin.attrib["filename"] = "libtobas_gazebo_noisydepth_plugin.so"

    ET.SubElement(plugin, "cameraName").text = "depth_camera"

    ET.SubElement(plugin, "irImageTopic").text = "ir/image_raw"
    ET.SubElement(plugin, "irInfoTopic").text = "ir/camera_info"
    ET.SubElement(plugin, "depthImageTopic").text = "depth/image_raw"
    ET.SubElement(plugin, "depthInfoTopic").text = "depth/camera_info"

    ET.SubElement(plugin, "distortionK1").text = "0.0"
    ET.SubElement(plugin, "distortionK2").text = "0.0"
    ET.SubElement(plugin, "distortionK3").text = "0.0"
    ET.SubElement(plugin, "distortionT1").text = "0.0"
    ET.SubElement(plugin, "distortionT2").text = "0.0"

    ET.SubElement(plugin, "robotNamespace").text = ns
    ET.SubElement(plugin, "frameName").text = optical_link
    ET.SubElement(plugin, "depthNoiseModel").text = noise_model
    ET.SubElement(plugin, "depthNoiseMinDist").text = str(near)
    ET.SubElement(plugin, "depthNoiseMaxDist").text = str(far)
    ET.SubElement(plugin, "horizontalFOV").text = str(fov)
    ET.SubElement(plugin, "baseline").text = str(baseline)
