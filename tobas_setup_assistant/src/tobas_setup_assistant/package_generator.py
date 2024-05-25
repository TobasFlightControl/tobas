from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant
    from .setting_widgets.propulsion_system.selected_links import SelectedLinkTabWidget

import os
import os.path as osp
import yaml
import rospy
import rospkg
import shutil
from xml.etree import ElementTree as ET
from jinja2 import Environment, FileSystemLoader
from PyQt5.QtCore import QObject, pyqtSlot
from PyQt5.QtWidgets import QMessageBox

from tobas_std_tools_py.sequence import is_unique
from tobas_std_tools_py.file import create_empty_file
from tobas_urdf_tools_py.core import *
from tobas_urdf_tools_py.gazebo import GazeboRosControl
from tobas_rqt_tools.path import resolve_uri
from tobas_rqt_tools.messages import q_info, q_error
from tobas_rqt_tools.xml import prettify_and_save
from tobas_tools_py.constants import CONTROLLER_NODE_NAME, OBSERVER_NODE_NAME
from tobas_msgs.msg import *

from .common import PKG_NAME
from .utils import get_drone_name
from .xml_nodes import *


class PackageGenerator(QObject):
    def __init__(self, main: SetupAssistant):
        super().__init__()
        self._main = main

        templates_path = osp.join(rospkg.RosPack().get_path(PKG_NAME), "templates")
        self._template_env = Environment(loader=FileSystemLoader(templates_path), trim_blocks=True, lstrip_blocks=True)

        self._drone_name = ""

    def define_connections(self) -> None:
        self._main.urdf_parser.robot_model_updated.connect(self._on_robot_model_updated)
        self._main.settings.ros_package.generate_button.clicked.connect(self._on_generate_button_clicked)

    @pyqtSlot()
    def _on_robot_model_updated(self) -> None:
        self._drone_name = get_drone_name()

    @pyqtSlot()
    def _on_generate_button_clicked(self) -> None:
        if not self._is_valid_config():
            return
        self._generate_pkg()
        q_info(self._main, "Configuration package is generated.")

    def _is_valid_config(self) -> bool:
        if not self._main.settings.start.is_valid():
            self._main.settings.switch(self._main.settings.start)
            return False
        if not self._main.settings.battery.is_valid():
            self._main.settings.switch(self._main.settings.battery)
            return False
        if not self._main.settings.propulsion_system.is_valid():
            self._main.settings.switch(self._main.settings.propulsion_system)
            return False
        if not self._main.settings.fixed_wing.is_valid():
            self._main.settings.switch(self._main.settings.fixed_wing)
            return False
        if not self._main.settings.custom_joints.is_valid():
            self._main.settings.switch(self._main.settings.custom_joints)
            return False
        if not self._main.settings.imu.is_valid():
            self._main.settings.switch(self._main.settings.imu)
            return False
        if not self._main.settings.barometer.is_valid():
            self._main.settings.switch(self._main.settings.barometer)
            return False
        if not self._main.settings.gps.is_valid():
            self._main.settings.switch(self._main.settings.gps)
            return False
        if not self._main.settings.rgb_camera.is_valid():
            self._main.settings.switch(self._main.settings.rgb_camera)
            return False
        if not self._main.settings.depth_camera.is_valid():
            self._main.settings.switch(self._main.settings.depth_camera)
            return False
        if not self._main.settings.lidar.is_valid():
            self._main.settings.switch(self._main.settings.lidar)
            return False
        if not self._main.settings.odometry.is_valid():
            self._main.settings.switch(self._main.settings.odometry)
            return False
        if not self._main.settings.tether_station.is_valid():
            self._main.settings.switch(self._main.settings.tether_station)
            return False
        if not self._main.settings.controller.is_valid():
            self._main.settings.switch(self._main.settings.controller)
            return False
        if not self._main.settings.observer.is_valid():
            self._main.settings.switch(self._main.settings.observer)
            return False
        if not self._main.settings.simulation.is_valid():
            self._main.settings.switch(self._main.settings.simulation)
            return False
        if not self._main.settings.author_information.is_valid():
            self._main.settings.switch(self._main.settings.author_information)
            return False
        if not self._main.settings.ros_package.is_valid():
            self._main.settings.switch(self._main.settings.ros_package)
            return False

        # Propulsion System, Control Surfaces, Custom Jointsの関節名が重複していないことを保証
        prop_jnt_names = self._main.settings.propulsion_system.selected.joint_names()
        cs_jnt_names = self._main.settings.fixed_wing.control_surfaces.selected.get_joint_names()
        custom_jnt_names = self._main.settings.custom_joints.joint_names()
        if not is_unique(prop_jnt_names + cs_jnt_names + custom_jnt_names):
            q_error(
                self, "The joints set in the propulsion system, control surfaces, and custom joints are duplicated."
            )
            return False

        return True

    def _generate_pkg(self) -> None:
        # 各ディレクトリのパス
        pkg_name = self._main.settings.ros_package.pkg_name.get()
        pkg_path = self._main.settings.ros_package.pkg_path()
        config_dir = osp.join(pkg_path, "config")
        launch_dir = osp.join(pkg_path, "launch")
        include_dir = osp.join(pkg_path, "include", pkg_name)
        src_dir = osp.join(pkg_path, "src")
        nodes_dir = osp.join(pkg_path, "nodes")
        nodelets_dir = osp.join(pkg_path, "nodelets")
        urdf_dir = osp.join(pkg_path, "urdf")
        mesh_dir = osp.join(pkg_path, "mesh")

        # ディレクトリを作る
        os.mkdir(pkg_path)
        os.mkdir(config_dir)
        os.mkdir(launch_dir)
        os.makedirs(include_dir)
        os.mkdir(src_dir)
        os.mkdir(nodes_dir)
        os.mkdir(nodelets_dir)
        os.mkdir(urdf_dir)
        os.mkdir(mesh_dir)

        # テンプレートから生成
        items = self._make_template_items()
        self._generate_from_template(items, "CMakeLists.txt.template", osp.join(pkg_path, "CMakeLists.txt"))
        self._generate_from_template(items, "package.xml.template", osp.join(pkg_path, "package.xml"))
        self._generate_from_template(
            items, "nodelet_description.xml.template", osp.join(pkg_path, "nodelet_description.xml")
        )
        self._generate_from_template(
            items, "plotjuggler_layout.xml.template", osp.join(config_dir, "plotjuggler_layout.xml")
        )
        self._generate_from_template(
            items, "nodelet_manager.launch.template", osp.join(launch_dir, "nodelet_manager.launch")
        )
        self._generate_from_template(
            items, "common_params.launch.template", osp.join(launch_dir, "common_params.launch")
        )
        self._generate_from_template(items, "gazebo.launch.template", osp.join(launch_dir, "gazebo.launch"))
        self._generate_from_template(items, "real.launch.template", osp.join(launch_dir, "real.launch"))
        self._generate_from_template(items, "calibration.launch.template", osp.join(launch_dir, "calibration.launch"))
        self._generate_from_template(items, "controller.launch.template", osp.join(launch_dir, "controller.launch"))
        self._generate_from_template(items, "observer.launch.template", osp.join(launch_dir, "observer.launch"))
        self._generate_from_template(
            items, "mission_action_servers.launch.template", osp.join(launch_dir, "mission_action_servers.launch")
        )
        self._generate_from_template(items, "bringup.launch.template", osp.join(launch_dir, "bringup.launch"))
        self._generate_from_template(items, "hil.launch.template", osp.join(launch_dir, "hil.launch"))
        self._generate_from_template(items, "rc_teleop.launch.template", osp.join(launch_dir, "rc_teleop.launch"))
        self._generate_from_template(
            items, "joint_control.launch.template", osp.join(launch_dir, "joint_control.launch")
        )
        self._generate_from_template(
            items, "jointpos_commander.launch.template", osp.join(launch_dir, "jointpos_commander.launch")
        )
        self._generate_from_template(items, "plotjuggler.launch.template", osp.join(launch_dir, "plotjuggler.launch"))
        self._generate_from_template(
            items, "motor_test_driver.launch.template", osp.join(launch_dir, "motor_test_driver.launch")
        )
        self._generate_from_template(
            items, "motor_test_gui.launch.template", osp.join(launch_dir, "motor_test_gui.launch")
        )
        self._generate_from_template(items, "tobas_bridge.launch.template", osp.join(launch_dir, "tobas_bridge.launch"))
        self._generate_from_template(items, "tobas_bridge.hpp.template", osp.join(include_dir, "tobas_bridge.hpp"))
        self._generate_from_template(items, "tobas_bridge.cpp.template", osp.join(src_dir, "tobas_bridge.cpp"))
        self._generate_from_template(
            items, "tobas_bridge_node.cpp.template", osp.join(nodes_dir, "tobas_bridge_node.cpp")
        )
        self._generate_from_template(
            items, "tobas_bridge_nodelet.hpp.template", osp.join(nodelets_dir, "tobas_bridge_nodelet.hpp")
        )
        self._generate_from_template(
            items, "tobas_bridge_nodelet.cpp.template", osp.join(nodelets_dir, "tobas_bridge_nodelet.cpp")
        )

        flight_modes = {self._main.settings.controller.stabilize_mode(), self._main.settings.controller.acrobat_mode()}

        # Keyboard Teleop (コントローラの対応コマンドによって場合分け)
        if PositionYaw.__name__ in flight_modes or PosVelAccYaw.__name__ in flight_modes:
            self._generate_from_template(
                items, "keyboard_teleop/position_yaw.launch.template", osp.join(launch_dir, "keyboard_teleop.launch")
            )
        elif SpeedRollDeltaPitch.__name__ in flight_modes:
            self._generate_from_template(
                items,
                "keyboard_teleop/speed_roll_dpitch.launch.template",
                osp.join(launch_dir, "keyboard_teleop.launch"),
            )

        # GUI Teleop (コントローラの対応コマンドによって場合分け)
        if (
            PositionYaw.__name__ in flight_modes
            or PosVelAccYaw.__name__ in flight_modes
            or PoseTwistAccelCommand.__name__ in flight_modes
        ):
            self._generate_from_template(
                items, "gui_teleop/position_yaw.launch.template", osp.join(launch_dir, "gui_teleop.launch")
            )

        # Pythonで自動生成
        self._generate_drone_config(config_dir)
        self._generate_joint_control_config(config_dir)
        self._generate_rc_teleop_config(config_dir)
        self._generate_controller_config(config_dir)
        self._generate_observer_config(config_dir)
        self._generate_dynamic_params_config(config_dir)
        self._generate_urdf(urdf_dir, mesh_dir)

    def _make_template_items(self) -> None:
        settings = self._main.settings

        template_items = dict()
        template_items["drone_name"] = self._drone_name

        # Controller
        controller = settings.controller
        template_items["controller_pkg"] = controller.controller_pkg()
        template_items["takeoff_pkg"] = controller.takeoff_pkg()
        template_items["landing_pkg"] = controller.landing_pkg()
        template_items["move_pkg"] = controller.move_pkg()

        # Observer
        template_items["observer_pkg"] = settings.observer.pkg_name()

        # Simulation
        simulation = settings.simulation
        template_items["gravity"] = simulation.gravity.get()

        # Author Info
        author_info = settings.author_information
        template_items["author_name"] = author_info.name.get()
        template_items["author_email"] = author_info.email.get()

        # Ros Package
        ros_pkg = settings.ros_package
        template_items["pkg_name"] = ros_pkg.pkg_name.get()

        # Joint Controllers
        custom_joints = settings.custom_joints
        joint_controllers = "joint_state_controller"
        for i in range(custom_joints.count()):
            jnt_name = custom_joints.joint_name(i)
            joint_controllers += f" {jnt_name}_controller"
        template_items["joint_controllers"] = joint_controllers

        return template_items

    def _generate_from_template(self, items: dict, template_file: str, out_path: str) -> None:
        template = self._template_env.get_template(template_file)
        content = template.render(items)  # テンプレートにdict型で文字を埋め込む
        with open(out_path, "w") as f:
            f.write(content)

    def _generate_drone_config(self, config_dir: str) -> None:
        # TBSFファイルに書き込むための辞書を作る
        drone_config = dict()

        # Drone Name
        drone_config["drone_name"] = self._drone_name

        # Battery
        battery = self._main.settings.battery
        drone_config["battery"] = {
            "nominal_voltage": battery.nominal_voltage(),
            "max_voltage": battery.max_voltage(),
            "sag_voltage": battery.sag_voltage(),
            "max_current": battery.max_current(),
        }

        # Propulsion System
        propulsion_system = self._main.settings.propulsion_system.selected

        num_rotors = propulsion_system.count()
        drone_config["num_rotors"] = num_rotors

        for i in range(num_rotors):
            selected: SelectedLinkTabWidget = propulsion_system.widget(i)

            # yamlに変換する際の文字化けを防ぐためにnp.float64から組み込みのfloatに変換
            drone_config[f"rotor_{i}"] = {
                "link_name": selected.link_name(),
                "direction": selected.motor.direction(),
                "axis": selected.axis_type(),
                "esc_signal_mode": selected.esc.signal_mode(),
                "num_poles": selected.motor.num_poles(),
                "max_rot_speed": float(selected.motor.max_rot_speed()),
                "rot_speed_coefs": [float(x) for x in selected.motor.rot_speed_coefs()],
                "time_constant_up": float(selected.motor.time_const_up()),
                "time_constant_down": float(selected.motor.time_const_down()),
                "motor_constant": float(selected.aerodynamics.motor_const()),
                "moment_constant": float(selected.aerodynamics.moment_const()),
                "drag_constant": float(selected.aerodynamics.rotor_drag_coef()),
                "channel": i,
            }

        # Fixed wing
        fixed_wing = self._main.settings.fixed_wing
        if fixed_wing.has_fixed_wing.isChecked():
            drone_config["fixed_wing"] = dict()

            # Vehicle
            vehicle = fixed_wing.vehicle
            drone_config["fixed_wing"]["vehicle"] = {
                "wing_surface": vehicle.wing_surface.get(),
                "wing_span": vehicle.wing_span.get(),
                "mean_aerodynamic_chord": vehicle.mac.get(),
                "aerodynamic_center": vehicle.aerodynamic_center.get(),
                "alpha_limit": {"lower": vehicle.alpha_limit.min(), "upper": vehicle.alpha_limit.max()},
            }

            # Aerodynamic Coefficients
            aero_coefs = fixed_wing.aero_coefs
            drone_config["fixed_wing"]["aerodynamic_coefficients"] = {
                "c_lift_0": aero_coefs.c_lift_0.value(),
                "c_lift_alpha": aero_coefs.c_lift_alpha.value(),
                "c_drag_0": aero_coefs.c_drag_0.value(),
                "c_drag_alpha": aero_coefs.c_drag_alpha.value(),
                "c_side_beta": aero_coefs.c_side_beta.value(),
                "c_roll_beta": aero_coefs.c_roll_beta.value(),
                "c_roll_p": aero_coefs.c_roll_p.value(),
                "c_roll_r": aero_coefs.c_roll_r.value(),
                "c_pitch_0": aero_coefs.c_pitch_0.value(),
                "c_pitch_alpha": aero_coefs.c_pitch_alpha.value(),
                "c_pitch_abs_beta": aero_coefs.c_pitch_abs_beta.value(),
                "c_pitch_alpha_rate": aero_coefs.c_pitch_alpha_rate.value(),
                "c_pitch_q": aero_coefs.c_pitch_q.value(),
                "c_yaw_beta": aero_coefs.c_yaw_beta.value(),
                "c_yaw_p": aero_coefs.c_yaw_p.value(),
                "c_yaw_r": aero_coefs.c_yaw_r.value(),
            }

            # Control Surfaces
            control_surfaces = fixed_wing.control_surfaces

            num_cs = fixed_wing.num_control_surfaces()
            drone_config["fixed_wing"]["num_control_surface"] = num_cs

            for idx, cs in enumerate(control_surfaces.control_surfaces()):
                drone_config["fixed_wing"][f"control_surface_{idx}"] = {
                    "angle_limit": {"lower": cs.min_angle, "upper": cs.max_angle},
                    "max_angle_rate": cs.max_angle_rate,
                    "c_lift_delta": cs.c_lift_delta,
                    "c_drag_abs_delta": cs.c_drag_abs_delta,
                    "c_side_delta": cs.c_side_delta,
                    "c_roll_delta": cs.c_roll_delta,
                    "c_pitch_delta": cs.c_pitch_delta,
                    "c_yaw_delta": cs.c_yaw_delta,
                }

        # Joints
        custom_joints = self._main.settings.custom_joints
        num_joints = custom_joints.count()
        drone_config["num_joints"] = num_joints
        for i in range(num_joints):
            drone_config[f"joint_{i}"] = {
                "name": custom_joints.joint_name(i),
                "home_position": custom_joints.home_position(i),
                "min_position": custom_joints.min_position(i),
                "max_position": custom_joints.max_position(i),
                "command_type": custom_joints.command_type(i),
            }

        # TBSFファイルを作成
        drone_config_path = osp.join(config_dir, "drone.tbsdrn")
        with open(drone_config_path, "w") as f:
            yaml.safe_dump(drone_config, f)

    def _generate_joint_control_config(self, config_dir: str) -> None:
        # yamlファイルに書き込むための辞書を作る
        items = dict()
        items["joint_state_controller"] = {"type": "joint_state_controller/JointStateController", "publish_rate": 1000}
        items["gazebo_ros_control"] = {"pid_gains": dict()}

        custom_joints = self._main.settings.custom_joints
        for i in range(custom_joints.count()):
            jnt_name = custom_joints.joint_name(i)
            controller_name = f"{jnt_name}_controller"
            items[controller_name] = {"joint": jnt_name, "type": custom_joints.controller_type(i)}

            if custom_joints.pid_enabled(i):
                items[controller_name]["pid"] = {
                    "p": custom_joints.p_gain(i),
                    "i": custom_joints.i_gain(i),
                    "d": custom_joints.d_gain(i),
                }
                items["gazebo_ros_control"]["pid_gains"][jnt_name] = {
                    "p": custom_joints.p_gain(i),
                    "i": custom_joints.i_gain(i),
                    "d": custom_joints.d_gain(i),
                }

        # yamlファイルを作成
        jnt_ctrl_path = osp.join(config_dir, "joint_control.yaml")
        with open(jnt_ctrl_path, "w") as f:
            yaml.safe_dump(items, f)

    def _generate_rc_teleop_config(self, config_dir: str) -> None:
        controller = self._main.settings.controller

        items = dict()
        items["rc_teleop"] = {"stabilize_mode": controller.stabilize_mode(), "acrobat_mode": controller.acrobat_mode()}

        file_path = osp.join(config_dir, "rc_teleop.yaml")
        with open(file_path, "w") as f:
            yaml.safe_dump(items, f)

    def _generate_controller_config(self, config_dir: str) -> None:
        items = {CONTROLLER_NODE_NAME: self._main.settings.controller.static_parameters()}
        file_path = osp.join(config_dir, "controller.yaml")
        with open(file_path, "w") as f:
            yaml.safe_dump(items, f)

    def _generate_observer_config(self, config_dir: str) -> None:
        items = {OBSERVER_NODE_NAME: self._main.settings.observer.static_parameters()}
        file_path = osp.join(config_dir, "observer.yaml")
        with open(file_path, "w") as f:
            yaml.safe_dump(items, f)

    def _generate_dynamic_params_config(self, config_dir: str) -> None:
        file_path = osp.join(config_dir, "dynamic_params.yaml")
        create_empty_file(file_path)

    def _generate_urdf(self, urdf_dir: str, mesh_dir: str) -> None:
        robot = self._make_urdf_with_plugins(mesh_dir)
        urdf_path = osp.join(urdf_dir, f"{self._drone_name}.xacro")

        # Save URDF
        # ET.ElementTree(robot).write(urdf_path)
        prettify_and_save(robot, urdf_path)

    def _make_urdf_with_plugins(self, mesh_dir: str) -> ET.Element:
        description = rospy.get_param("/robot_description")
        robot = ET.fromstring(description)
        assert robot.tag == "robot"

        self._resolve_mesh_files(robot, mesh_dir)
        self._screen_xml_elements(robot)
        self._add_xml_elements(robot)

        return robot

    def _resolve_mesh_files(self, robot: ET.Element, mesh_dir: str) -> None:
        """全てのメッシュファイルのパスをパッケージ以下に変更する．"""
        pkg_name = self._main.settings.ros_package.pkg_name.get()
        for mesh in robot.iter("mesh"):
            abs_path = resolve_uri(mesh.attrib["filename"])
            base_name = osp.basename(abs_path)
            shutil.copy2(abs_path, osp.join(mesh_dir, base_name))  # メッシュファイルをコピー
            mesh.attrib["filename"] = f"package://{pkg_name}/mesh/{base_name}"

    def _screen_xml_elements(self, robot: ET.Element) -> None:
        """悪影響を与えるかもしれないXML要素を，ユーザに確認した上で消す．"""
        # gazeboタグの場合はその子ノードを確認する
        for gazebo in robot.iter("gazebo"):
            for child in gazebo:
                if child.tag == "plugin":
                    # Tobasのプラグインは問答無用で消す
                    if child.attrib["filename"].startswith("libtobas"):
                        robot.remove(gazebo)
                        continue
                    # RotorSのプラグインは問答無用で消す
                    if child.attrib["filename"].startswith("librotors"):
                        robot.remove(gazebo)
                        continue
                    # Gazebo ROS Controlは問答無用で消す
                    if child.attrib["filename"] == "libgazebo_ros_control.so":
                        robot.remove(gazebo)
                        continue
                    self._remove_or_keep_gazebo_child(gazebo, child)
                elif child.tag == "sensor":
                    self._remove_or_keep_gazebo_child(gazebo, child)

    def _remove_or_keep_gazebo_child(self, gazebo: ET.Element, child: ET.Element) -> None:
        """属性を確認した上でGazeboの子ノードを削除する．"""
        msg_box = QMessageBox(self._main)  # 親を設定しておけば一緒に落とせる

        # テキストの設定
        text = f"Gazebo {child.tag} is detected.\n\n"
        for key, value in child.attrib.items():
            text += f"    {key}: {value}\n"
        text += "\nThis may interfere with components automatically added by Tobas."
        msg_box.setText(text)
        msg_box.setInformativeText(f"Do you remove this {child.tag} or keep it?")

        # ボタンの設定
        remove_button = msg_box.addButton("Remove", QMessageBox.ActionRole)
        keep_button = msg_box.addButton("Keep", QMessageBox.ActionRole)
        msg_box.setDefaultButton(remove_button)

        # ユーザの返事を取得
        msg_box.exec()

        # Removeが選択されたら消す
        if msg_box.clickedButton() == remove_button:
            gazebo.remove(child)

    def _add_xml_elements(self, robot: ET.Element) -> None:
        root_link = self._main.urdf_parser.get_root().name

        propulsion_system = self._main.settings.propulsion_system.selected
        fixed_wing = self._main.settings.fixed_wing
        battery = self._main.settings.battery
        imu = self._main.settings.imu
        barometer = self._main.settings.barometer
        gps = self._main.settings.gps
        rgb_camera = self._main.settings.rgb_camera
        depth_camera = self._main.settings.depth_camera
        lidar = self._main.settings.lidar
        odometry = self._main.settings.odometry
        tether_station = self._main.settings.tether_station
        simulation = self._main.settings.simulation

        # XML namespace
        robot.attrib["xmlns:xacro"] = "http://ros.org/wiki/xacro"

        # Base static joint for debug
        base_fix_joint = BaseStaticJoint(root_link=root_link)
        robot.append(base_fix_joint)

        # Base plugin
        base_plugin = BasePlugin(ns=self._drone_name, rotor_joint_names=propulsion_system.joint_names())
        robot.append(base_plugin)

        # Wind plugin
        wind_model = WindModel(ns=self._drone_name, link_name=root_link)
        robot.append(wind_model)

        # Battery plugin
        battery_model = BatteryModel(
            ns=self._drone_name,
            max_voltage=battery.max_voltage(),
            sag_voltage=battery.sag_voltage(),
            max_current=battery.max_current(),
            capacity=battery.capacity(),
            internal_registance=battery.internal_registance(),
            num_rotors=propulsion_system.count(),
        )
        robot.append(battery_model)

        # Propulsion System plugin
        for i in range(propulsion_system.count()):
            selected: SelectedLinkTabWidget = propulsion_system.widget(i)
            motor_model = MotorModel(
                ns=self._drone_name,
                motor_number=i,
                link_name=selected.link_name(),
                joint_name=selected.joint_name(),
                direction=selected.motor.direction(),
                rot_speed_coefs=selected.motor.rot_speed_coefs(),
                motor_const=selected.aerodynamics.motor_const(),
                moment_const=selected.aerodynamics.moment_const(),
                rotor_drag_coef=selected.aerodynamics.rotor_drag_coef(),
                max_model_error_rate=selected.aerodynamics.max_model_error_rate(),
                time_const_up=selected.motor.time_const_up(),
                time_const_down=selected.motor.time_const_down(),
                max_rot_speed=selected.motor.max_rot_speed(),
                max_current=selected.esc.max_current(),
            )
            robot.append(motor_model)

        # Fixed wing plugin
        if fixed_wing.has_fixed_wing.isChecked():
            vehicle = fixed_wing.vehicle
            aero_coefs = fixed_wing.aero_coefs
            control_surfaces = fixed_wing.control_surfaces
            fixed_wing_model = FixedWingModel(
                ns=self._drone_name,
                link_name=root_link,
                altitude_0=simulation.altitude_0.get(),
                wing_surface=vehicle.wing_surface.get(),
                wing_span=vehicle.wing_span.get(),
                mean_aerodynamic_chord=vehicle.mac.get(),
                aerodynamic_center=vehicle.aerodynamic_center.get(),
                alpha_limit=vehicle.alpha_limit.get(),
                c_lift_0=aero_coefs.c_lift_0.value(),
                c_lift_alpha=aero_coefs.c_lift_alpha.value(),
                c_drag_0=aero_coefs.c_drag_0.value(),
                c_drag_alpha=aero_coefs.c_drag_alpha.value(),
                c_side_beta=aero_coefs.c_side_beta.value(),
                c_roll_beta=aero_coefs.c_roll_beta.value(),
                c_roll_p=aero_coefs.c_roll_p.value(),
                c_roll_r=aero_coefs.c_roll_r.value(),
                c_pitch_0=aero_coefs.c_pitch_0.value(),
                c_pitch_alpha=aero_coefs.c_pitch_alpha.value(),
                c_pitch_abs_beta=aero_coefs.c_pitch_abs_beta.value(),
                c_pitch_alpha_rate=aero_coefs.c_pitch_alpha_rate.value(),
                c_pitch_q=aero_coefs.c_pitch_q.value(),
                c_yaw_beta=aero_coefs.c_yaw_beta.value(),
                c_yaw_p=aero_coefs.c_yaw_p.value(),
                c_yaw_r=aero_coefs.c_yaw_r.value(),
                control_surfaces=control_surfaces.control_surfaces(),
            )
            robot.append(fixed_wing_model)

        if imu.equipped():
            # IMU plugin
            imu_model = ImuModel(
                ns=self._drone_name,
                link_name=root_link,
                update_rate=imu.update_rate.get(),
                offset=imu.offset.get(),
                gyro_noise_density=imu.gyro_noise_density.get(),
                gyro_random_walk=imu.gyro_random_walk.get(),
                gyro_bias_corr_time=imu.gyro_bias_corr_time.get(),
                gyro_turn_on_bias_sigma=imu.gyro_turn_on_bias_sigma.get(),
                gyro_lpf_cutoff_freq=imu.gyro_lpf_cutoff_freq.get(),
                acc_noise_density=imu.acc_noise_density.get(),
                acc_random_walk=imu.acc_random_walk.get(),
                acc_bias_corr_time=imu.acc_bias_corr_time.get(),
                acc_turn_on_bias_sigma=imu.acc_turn_on_bias_sigma.get(),
                acc_lpf_cutoff_freq=imu.acc_lpf_cutoff_freq.get(),
            )
            robot.append(imu_model)

            # Magnetometer plugin
            mag_model = MagnetometerModel(
                ns=self._drone_name,
                link_name=root_link,
                update_rate=imu.update_rate.get(),
                offset=imu.offset.get(),
                latitude_0=simulation.latitude_0.get(),
                longitude_0=simulation.longitude_0.get(),
                altitude_0=simulation.altitude_0.get(),
                gauss_noise=imu.mag_gauss_noise.get(),
                uniform_noise=imu.mag_uniform_noise.get(),
            )
            robot.append(mag_model)

        # Barometer plugin
        if barometer.equipped():
            bar_model = BarometerModel(
                ns=self._drone_name,
                link_name=root_link,
                update_rate=barometer.update_rate.get(),
                offset=barometer.offset.get(),
                altitude_0=simulation.altitude_0.get(),
                pressure_var=barometer.pressure_var.get(),
            )
            robot.append(bar_model)

        # GPS plugin
        if gps.equipped():
            gps_model = GpsModel(
                ns=self._drone_name,
                link_name=root_link,
                offset=gps.offset.get(),
                update_rate=gps.update_rate.get(),
                delay=gps.delay.get(),
                pos_corr_time=gps.pos_corr_time.get(),
                hor_pos_accuracy=gps.horizontal_pos_accuracy.get(),
                ver_pos_accuracy=gps.vertical_pos_accuracy.get(),
                hor_vel_stddev=gps.horizontal_vel_stddev.get(),
                ver_vel_stddev=gps.vertical_vel_stddev.get(),
                latitude_0=simulation.latitude_0.get(),
                longitude_0=simulation.longitude_0.get(),
                altitude_0=simulation.altitude_0.get(),
            )
            robot.append(gps_model)

        # RGB Camera plugin
        if rgb_camera.equipped():
            add_rgb_camera_model(
                robot=robot,
                ns=self._drone_name,
                link_name=rgb_camera.link.get(),
                offset=Origin(
                    x=rgb_camera.offset.x(),
                    y=rgb_camera.offset.y(),
                    z=rgb_camera.offset.z(),
                    roll=rgb_camera.offset.roll(),
                    pitch=rgb_camera.offset.pitch(),
                    yaw=rgb_camera.offset.yaw(),
                ),
                frame_rate=rgb_camera.update_rate.get(),
                width=rgb_camera.image_width.get(),
                height=rgb_camera.image_height.get(),
                near=rgb_camera.depth_range.min(),
                far=rgb_camera.depth_range.max(),
                fov=rgb_camera.fov.get(),
                noise_mean=0.0,
                noise_stddev=rgb_camera.noise_stddev.get(),
            )

        # Depth Camera plugin
        if depth_camera.equipped():
            add_depth_camera_model(
                robot=robot,
                ns=self._drone_name,
                link_name=depth_camera.link.get(),
                offset=Origin(
                    x=depth_camera.offset.x(),
                    y=depth_camera.offset.y(),
                    z=depth_camera.offset.z(),
                    roll=depth_camera.offset.roll(),
                    pitch=depth_camera.offset.pitch(),
                    yaw=depth_camera.offset.yaw(),
                ),
                frame_rate=depth_camera.update_rate.get(),
                width=depth_camera.image_width.get(),
                height=depth_camera.image_height.get(),
                near=depth_camera.depth_range.min(),
                far=depth_camera.depth_range.max(),
                fov=depth_camera.fov.get(),
                baseline=depth_camera.baseline.get(),
                noise_model=depth_camera.noise_model.get(),
            )

        # LiDAR plugin
        if lidar.equipped():
            add_lidar_model(
                robot=robot,
                ns=self._drone_name,
                link_name=root_link,
                offset=Origin.Trans(lidar.offset.x(), lidar.offset.y(), lidar.offset.z()),
                update_rate=lidar.update_rate.get(),
                hor_samples=lidar.hor_samples.get(),
                ver_samples=lidar.ver_samples.get(),
                hor_fov=lidar.hor_fov.get(),
                ver_fov=lidar.ver_fov.get(),
                dist_range=lidar.range.get(),
                resolution=lidar.resolution.get(),
                noise_stddev=lidar.noise_stddev.get(),
            )

        # Odometry plugin
        if odometry.equipped():
            odometry_model = OdometryModel(
                ns=self._drone_name,
                link_name=root_link,
                update_rate=odometry.update_rate.get(),
                offset=odometry.offset.get(),
                pos_normal_noise_std=odometry.pos_normal_noise_std.get(),
                rot_normal_noise_std=odometry.rot_normal_noise_std.get(),
                linvel_normal_noise_std=odometry.linvel_normal_noise_std.get(),
                angvel_normal_noise_std=odometry.angvel_normal_noise_std.get(),
                pos_uniform_noise_scale=odometry.pos_uniform_noise_scale.get(),
                rot_uniform_noise_scale=odometry.rot_uniform_noise_scale.get(),
                linvel_uniform_noise_scale=odometry.linvel_uniform_noise_scale.get(),
                angvel_uniform_noise_scale=odometry.angvel_uniform_noise_scale.get(),
            )
            robot.append(odometry_model)

        # Tether Station plugin
        if tether_station.equipped():
            add_tether_station_model(
                robot=robot,
                link_name=tether_station.link.get(),
                world_end=tether_station.world_end.get(),
                drone_end=tether_station.drone_end.get(),
                tension=tether_station.tension.get(),
            )

        # Ground Truth State plugin
        state_gt_model = GroundTruthStateModel(self._drone_name, root_link)
        robot.append(state_gt_model)

        # ROS Control
        ros_control = GazeboRosControl(self._drone_name)
        robot.append(ros_control)
