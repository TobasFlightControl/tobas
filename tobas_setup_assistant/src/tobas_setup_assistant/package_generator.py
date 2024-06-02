from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant
    from .setting_widgets.propulsion_system.selected_links import SelectedLinkWidget

import os
import os.path as osp
import yaml
import rospy
import rospkg
import shutil
from xml.etree import ElementTree as ET
from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import QMessageBox

from tobas_std_tools_py.sequence import is_unique
from tobas_std_tools_py.file import create_empty_file
from tobas_urdf_tools_py.core import Origin
from tobas_urdf_tools_py.gazebo import GazeboRosControl
from tobas_urdf_tools_py.utils import prettify
from tobas_rqt_tools.widgets import ProgressDialog
from tobas_rqt_tools.path import resolve_uri
from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.constants import CONTROLLER_NODE_NAME, OBSERVER_NODE_NAME
from tobas_tools_py.command import build_tobas_package
from tobas_tools_py.package import *
from tobas_msgs.msg import PositionYaw, PosVelAccYaw, PoseTwistAccelCommand, SpeedRollDeltaPitch

from .common import PKG_NAME
from .utils import get_drone_name, TemplateGenerator
from .xml_nodes import *


class PackageGenerator(QObject):
    def __init__(self, main: SetupAssistant):
        super().__init__()
        self._main = main

        templates_path = osp.join(rospkg.RosPack().get_path(PKG_NAME), "templates")
        self._meta_env = TemplateGenerator(osp.join(templates_path, "meta_package"))
        self._cfg_env = TemplateGenerator(osp.join(templates_path, "config_package"))
        self._usr_env = TemplateGenerator(osp.join(templates_path, "user_package"))

        self._drone_name = ""

    def update_internal_data_structures(self) -> None:
        self._drone_name = get_drone_name()

    def generate_package(self) -> None:
        progress = ProgressDialog(parent=self._main, title="Generate Package", num_steps=3)
        progress.setCancelButton(None)
        progress.show()

        progress.setLabelText("Verifying the validity of the user settings.")
        if not self._is_valid_config():
            progress.close()
            return
        progress.progress_step()

        progress.setLabelText("Generating Tobas packages.")
        try:
            self._generate_pkg()
        except Exception as e:
            progress.close()
            q_error(self._main, f"A proglem ocurred while generating Tobas packages:\n\n{e}")
            return
        progress.progress_step()

        # Build Tobas package
        progress.setLabelText("Building Tobas packages.")
        if not build_tobas_package(self._main.ros_package.pkg_path()):
            progress.close()
            q_error(self._main, "Failed to build Tobas package.")
            return
        progress.progress_step()

        progress.close()
        q_info(self._main, "Tobas configuration package is generated and built successfully.")

    def _is_valid_config(self) -> bool:
        # 全ての設定項目について，単体で問題ないことを確認
        for i in range(self._main.num_setting_widgets()):
            setting_widget = self._main.get_setting_widget(i)
            if not setting_widget.is_valid():
                self._main.switch(setting_widget)
                return False

        # Propulsion System, Control Surfaces, Custom Jointsの関節名が重複していないことを確認
        prop_jnt_names = self._main.propulsion_system.selected.joint_names()
        cs_jnt_names = self._main.fixed_wing.control_surfaces.selected.get_joint_names()
        custom_jnt_names = self._main.custom_joints.get_joint_names()
        if not is_unique(prop_jnt_names + cs_jnt_names + custom_jnt_names):
            q_error(
                self, "The joints set in the propulsion system, control surfaces, and custom joints are duplicated."
            )
            return False

        return True

    def _generate_pkg(self) -> None:
        tbs_path = self._main.ros_package.pkg_path()

        # Tobasパッケージを作成
        os.makedirs(tbs_path, exist_ok=True)

        # テンプレート用アイテムを作成
        items = self._make_template_items()

        # メタパッケージを作成
        self._generate_meta_pkg(items)

        # 設定パッケージを作成
        self._generate_config_pkg(items)

        # ユーザパッケージを作成
        self._generate_user_pkg(items)

    def _generate_meta_pkg(self, items: dict) -> None:
        meta_pkg_path = get_tbs_meta_path(self._main.ros_package.pkg_path())
        os.makedirs(meta_pkg_path, exist_ok=True)

        self._meta_env.generate(items, "CMakeLists.txt.tpl", meta_pkg_path)
        self._meta_env.generate(items, "package.xml.tpl", meta_pkg_path)

        create_empty_file(osp.join(meta_pkg_path, "DO_NOT_EDIT_THIS_PACKAGE"), exist_ok=True)

    def _generate_config_pkg(self, items: dict) -> None:
        config_pkg_path = get_tbs_config_path(self._main.ros_package.pkg_path())
        os.makedirs(config_pkg_path, exist_ok=True)

        # ディレクトリを作成
        config_dir = osp.join(config_pkg_path, "config")
        launch_dir = osp.join(config_pkg_path, "launch")
        urdf_dir = osp.join(config_pkg_path, "urdf")
        mesh_dir = osp.join(config_pkg_path, "mesh")
        os.makedirs(config_dir, exist_ok=True)
        os.makedirs(launch_dir, exist_ok=True)
        os.makedirs(urdf_dir, exist_ok=True)
        os.makedirs(mesh_dir, exist_ok=True)

        # バックアップ用ファイル
        self._dump_settings()

        # テンプレートから生成
        self._cfg_env.generate(items, "CMakeLists.txt.tpl", config_pkg_path)
        self._cfg_env.generate(items, "package.xml.tpl", config_pkg_path)
        self._cfg_env.generate(items, "plotjuggler_layout.xml.tpl", config_dir)
        self._cfg_env.generate(items, "nodelet_manager.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "common_params.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "gazebo.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "real.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "calibration.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "controller.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "observer.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "mission_action_servers.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "bringup.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "hil.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "rc_teleop.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "joint_control.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "jointpos_commander.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "plotjuggler.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "motor_test_driver.launch.tpl", launch_dir)
        self._cfg_env.generate(items, "motor_test_gui.launch.tpl", launch_dir)

        flight_modes = {self._main.controller.stabilize_mode(), self._main.controller.acrobat_mode()}

        # Keyboard Teleop (コントローラの対応コマンドによって場合分け)
        if PositionYaw.__name__ in flight_modes or PosVelAccYaw.__name__ in flight_modes:
            self._cfg_env.generate(items, "keyboard_teleop/position_yaw/keyboard_teleop.launch.tpl", launch_dir)
        elif SpeedRollDeltaPitch.__name__ in flight_modes:
            self._cfg_env.generate(items, "keyboard_teleop/speed_roll_dpitch/keyboard_teleop.launch.tpl", launch_dir)

        # GUI Teleop (コントローラの対応コマンドによって場合分け)
        if (
            PositionYaw.__name__ in flight_modes
            or PosVelAccYaw.__name__ in flight_modes
            or PoseTwistAccelCommand.__name__ in flight_modes
        ):
            self._cfg_env.generate(items, "gui_teleop/position_yaw/gui_teleop.launch.tpl", launch_dir)

        # その他
        create_empty_file(osp.join(config_pkg_path, "DO_NOT_EDIT_THIS_PACKAGE"), exist_ok=True)
        create_empty_file(osp.join(config_dir, "dynamic_params.yaml"), exist_ok=True)
        self._generate_drone_config(config_dir)
        self._generate_joint_control_config(config_dir)
        self._generate_rc_teleop_config(config_dir)
        self._generate_controller_config(config_dir)
        self._generate_observer_config(config_dir)
        self._generate_urdf(urdf_dir, mesh_dir)

    def _generate_user_pkg(self, items: dict) -> None:
        user_pkg_name = get_tbs_user_name(self._main.ros_package.pkg_path())
        user_pkg_path = get_tbs_user_path(self._main.ros_package.pkg_path())
        os.makedirs(user_pkg_path, exist_ok=True)

        # ディレクトリを作成
        launch_dir = osp.join(user_pkg_path, "launch")
        include_dir = osp.join(user_pkg_path, "include", user_pkg_name)
        src_dir = osp.join(user_pkg_path, "src")
        nodes_dir = osp.join(user_pkg_path, "nodes")
        nodelets_dir = osp.join(user_pkg_path, "nodelets")
        os.makedirs(launch_dir, exist_ok=True)
        os.makedirs(include_dir, exist_ok=True)
        os.makedirs(src_dir, exist_ok=True)
        os.makedirs(nodes_dir, exist_ok=True)
        os.makedirs(nodelets_dir, exist_ok=True)

        # テンプレートから作成 (存在する場合は上書きしない)
        self._usr_env.generate(items, "CMakeLists.txt.tpl", user_pkg_path, overwrite=False)
        self._usr_env.generate(items, "package.xml.tpl", user_pkg_path, overwrite=False)
        self._usr_env.generate(items, "common.launch.tpl", launch_dir, overwrite=False)
        self._usr_env.generate(items, "gazebo.launch.tpl", launch_dir, overwrite=False)
        self._usr_env.generate(items, "real.launch.tpl", launch_dir, overwrite=False)
        self._usr_env.generate(items, "nodelet_description.xml.tpl", user_pkg_path, overwrite=False)
        self._usr_env.generate(items, "tobas_bridge.hpp.tpl", include_dir, overwrite=False)
        self._usr_env.generate(items, "tobas_bridge.cpp.tpl", src_dir, overwrite=False)
        self._usr_env.generate(items, "tobas_bridge_node.cpp.tpl", nodes_dir, overwrite=False)
        self._usr_env.generate(items, "tobas_bridge_nodelet.hpp.tpl", nodelets_dir, overwrite=False)
        self._usr_env.generate(items, "tobas_bridge_nodelet.cpp.tpl", nodelets_dir, overwrite=False)

        # その他
        create_empty_file(osp.join(user_pkg_path, "YOU_CAN_EDIT_THIS_PACKAGE"), exist_ok=True)

    def _dump_settings(self) -> None:
        # データを作成
        data = dict()
        for i in range(self._main.num_setting_widgets()):
            setting_widget = self._main.get_setting_widget(i)
            data[setting_widget.NAME] = setting_widget.dump_settings()

        # yaml形式で保存
        settings_path = get_settings_path(self._main.ros_package.pkg_path())
        with open(settings_path, "w") as f:
            yaml.safe_dump(data, f)

    def _make_template_items(self) -> dict:
        template_items = dict()
        template_items["drone_name"] = self._drone_name

        # Controller
        template_items["controller_pkg"] = self._main.controller.controller_pkg()
        template_items["takeoff_pkg"] = self._main.controller.takeoff_pkg()
        template_items["landing_pkg"] = self._main.controller.landing_pkg()
        template_items["move_pkg"] = self._main.controller.move_pkg()

        # Observer
        template_items["observer_pkg"] = self._main.observer.pkg_name()

        # Simulation
        template_items["gravity"] = self._main.simulation.gravity.get()

        # Author Info
        template_items["author_name"] = self._main.author_information.name.get()
        template_items["author_email"] = self._main.author_information.email.get()

        # Ros Package
        template_items["meta_pkg_name"] = get_tbs_meta_name(self._main.ros_package.pkg_path())
        template_items["config_pkg_name"] = get_tbs_config_name(self._main.ros_package.pkg_path())
        template_items["user_pkg_name"] = get_tbs_user_name(self._main.ros_package.pkg_path())

        # Joint Controllers
        joint_controllers = "joint_state_controller"
        for i in range(self._main.custom_joints.count()):
            jnt_name = self._main.custom_joints.get_joint_name(i)
            joint_controllers += f" {jnt_name}_controller"
        template_items["joint_controllers"] = joint_controllers

        return template_items

    def _generate_drone_config(self, config_dir: str) -> None:
        # TBSFファイルに書き込むための辞書を作る
        drone_config = dict()

        # Drone Name
        drone_config["drone_name"] = self._drone_name

        # Battery
        drone_config["battery"] = {
            "nominal_voltage": self._main.battery.nominal_voltage(),
            "max_voltage": self._main.battery.max_voltage(),
            "sag_voltage": self._main.battery.sag_voltage(),
            "max_current": self._main.battery.max_current(),
        }

        # Propulsion System
        selected_props = self._main.propulsion_system.selected

        num_rotors = selected_props.count()
        drone_config["num_rotors"] = num_rotors

        for i in range(num_rotors):
            selected: SelectedLinkWidget = selected_props.widget(i)

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
        if self._main.fixed_wing.has_fixed_wing.isChecked():
            drone_config["fixed_wing"] = dict()

            # Vehicle
            vehicle = self._main.fixed_wing.vehicle
            drone_config["fixed_wing"]["vehicle"] = {
                "wing_surface": vehicle.wing_surface.get(),
                "wing_span": vehicle.wing_span.get(),
                "mean_aerodynamic_chord": vehicle.mac.get(),
                "aerodynamic_center": vehicle.aerodynamic_center.get(),
                "alpha_limit": {"lower": vehicle.alpha_limit.min(), "upper": vehicle.alpha_limit.max()},
            }

            # Aerodynamic Coefficients
            aero_coefs = self._main.fixed_wing.aero_coefs
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
            num_cs = self._main.fixed_wing.num_control_surfaces()
            drone_config["fixed_wing"]["num_control_surface"] = num_cs

            for idx, cs in enumerate(self._main.fixed_wing.control_surfaces.control_surfaces()):
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
        num_joints = self._main.custom_joints.count()
        drone_config["num_joints"] = num_joints
        for i in range(num_joints):
            drone_config[f"joint_{i}"] = {
                "name": self._main.custom_joints.get_joint_name(i),
                "home_position": self._main.custom_joints.get_home_position(i),
                "min_position": self._main.custom_joints.get_min_position(i),
                "max_position": self._main.custom_joints.get_max_position(i),
                "command_type": self._main.custom_joints.get_command_type(i),
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

        for i in range(self._main.custom_joints.count()):
            jnt_name = self._main.custom_joints.get_joint_name(i)
            controller_name = f"{jnt_name}_controller"
            items[controller_name] = {"joint": jnt_name, "type": self._main.custom_joints.get_controller_type(i)}

            if self._main.custom_joints.pid_enabled(i):
                items[controller_name]["pid"] = {
                    "p": self._main.custom_joints.get_p_gain(i),
                    "i": self._main.custom_joints.get_i_gain(i),
                    "d": self._main.custom_joints.get_d_gain(i),
                }
                items["gazebo_ros_control"]["pid_gains"][jnt_name] = {
                    "p": self._main.custom_joints.get_p_gain(i),
                    "i": self._main.custom_joints.get_i_gain(i),
                    "d": self._main.custom_joints.get_d_gain(i),
                }

        # yamlファイルを作成
        jnt_ctrl_path = osp.join(config_dir, "joint_control.yaml")
        with open(jnt_ctrl_path, "w") as f:
            yaml.safe_dump(items, f)

    def _generate_rc_teleop_config(self, config_dir: str) -> None:
        controller = self._main.controller

        items = dict()
        items["rc_teleop"] = {"stabilize_mode": controller.stabilize_mode(), "acrobat_mode": controller.acrobat_mode()}

        file_path = osp.join(config_dir, "rc_teleop.yaml")
        with open(file_path, "w") as f:
            yaml.safe_dump(items, f)

    def _generate_controller_config(self, config_dir: str) -> None:
        items = {CONTROLLER_NODE_NAME: self._main.controller.static_parameters()}
        file_path = osp.join(config_dir, "controller.yaml")
        with open(file_path, "w") as f:
            yaml.safe_dump(items, f)

    def _generate_observer_config(self, config_dir: str) -> None:
        items = {OBSERVER_NODE_NAME: self._main.observer.static_parameters()}
        file_path = osp.join(config_dir, "observer.yaml")
        with open(file_path, "w") as f:
            yaml.safe_dump(items, f)

    def _generate_urdf(self, urdf_dir: str, mesh_dir: str) -> None:
        robot = self._make_urdf_with_plugins(mesh_dir)
        urdf_path = osp.join(urdf_dir, f"drone.xacro")

        # Save URDF
        with open(urdf_path, "w") as f:
            f.write(prettify(robot))

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
        config_pkg_name = get_tbs_config_name(self._main.ros_package.pkg_path())
        for mesh in robot.iter("mesh"):
            src_path = resolve_uri(mesh.attrib["filename"])
            base_name = osp.basename(src_path)
            des_path = osp.join(mesh_dir, base_name)
            if src_path != des_path:
                shutil.copy2(src_path, des_path)  # メッシュファイルをコピー
                mesh.attrib["filename"] = f"package://{config_pkg_name}/mesh/{base_name}"

    def _screen_xml_elements(self, robot: ET.Element) -> None:
        """悪影響を与えるかもしれないXML要素を，ユーザに確認した上で消す．"""
        # 繰り返し中にツリー構造を変えるとバグるため，消すノードをリストしておいて後で消す
        deletable_nodes = []

        # 全てのGazeboノードを走査し，消すべきノードをリスト
        for child in robot:
            if child.tag == "gazebo" and self._is_deletable_gazebo_node(child):
                deletable_nodes.append(child)

        # リストしたGazeboノードを削除
        for node in deletable_nodes:
            robot.remove(node)

    def _is_deletable_gazebo_node(self, gazebo: ET.Element) -> bool:
        assert gazebo.tag == "gazebo"

        for child in gazebo:
            if child.tag == "plugin":
                if self._is_deletable_gazebo_plugin(child) or self._ask_remove_or_keep_gazebo_child(child):
                    return True
            elif child.tag == "sensor":
                for plugin in child.iter("plugin"):
                    if self._is_deletable_gazebo_plugin(plugin) or self._ask_remove_or_keep_gazebo_child(child):
                        return True

        return False

    def _is_deletable_gazebo_plugin(self, plugin: ET.Element) -> bool:
        """プラグインを強制削除すべきかどうかを判定する．"""
        assert plugin.tag == "plugin"

        filename = plugin.attrib["filename"]
        return (
            filename.startswith("libtobas")
            or filename.startswith("librotors")
            or filename == "libgazebo_ros_control.so"
        )

    def _ask_remove_or_keep_gazebo_child(self, child: ET.Element) -> bool:
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
        msg_box.addButton("Keep", QMessageBox.ActionRole)
        msg_box.setDefaultButton(remove_button)

        # ユーザの返事を取得
        msg_box.exec()

        # Removeが選択されたらTrue
        return msg_box.clickedButton() == remove_button

    def _add_xml_elements(self, robot: ET.Element) -> None:
        root_link = self._main.urdf_parser.get_root().name

        # XML namespace
        robot.attrib["xmlns:xacro"] = "http://ros.org/wiki/xacro"

        # Base static joint for debug
        base_fix_joint = BaseStaticJoint(root_link=root_link)
        robot.append(base_fix_joint)

        # Base plugin
        base_plugin = BasePlugin(
            ns=self._drone_name, rotor_joint_names=self._main.propulsion_system.selected.joint_names()
        )
        robot.append(base_plugin)

        # Wind plugin
        wind_model = WindModel(ns=self._drone_name, link_name=root_link)
        robot.append(wind_model)

        # Battery plugin
        battery_model = BatteryModel(
            ns=self._drone_name,
            max_voltage=self._main.battery.max_voltage(),
            sag_voltage=self._main.battery.sag_voltage(),
            max_current=self._main.battery.max_current(),
            capacity=self._main.battery.capacity(),
            internal_registance=self._main.battery.internal_registance(),
            num_rotors=self._main.propulsion_system.selected.count(),
        )
        robot.append(battery_model)

        # Propulsion System plugin
        for i in range(self._main.propulsion_system.selected.count()):
            selected: SelectedLinkWidget = self._main.propulsion_system.selected.widget(i)
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
        if self._main.fixed_wing.has_fixed_wing.isChecked():
            vehicle = self._main.fixed_wing.vehicle
            aero_coefs = self._main.fixed_wing.aero_coefs
            control_surfaces = self._main.fixed_wing.control_surfaces
            fixed_wing_model = FixedWingModel(
                ns=self._drone_name,
                link_name=root_link,
                altitude_0=self._main.simulation.altitude_0.get(),
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

        if self._main.imu.equipped():
            # IMU plugin
            imu_model = ImuModel(
                ns=self._drone_name,
                link_name=root_link,
                update_rate=self._main.imu.update_rate.get(),
                offset=self._main.imu.offset.get(),
                gyro_noise_density=self._main.imu.gyro_noise_density.get(),
                gyro_random_walk=self._main.imu.gyro_random_walk.get(),
                gyro_bias_corr_time=self._main.imu.gyro_bias_corr_time.get(),
                gyro_turn_on_bias_sigma=self._main.imu.gyro_turn_on_bias_sigma.get(),
                gyro_lpf_cutoff_freq=self._main.imu.gyro_lpf_cutoff_freq.get(),
                acc_noise_density=self._main.imu.acc_noise_density.get(),
                acc_random_walk=self._main.imu.acc_random_walk.get(),
                acc_bias_corr_time=self._main.imu.acc_bias_corr_time.get(),
                acc_turn_on_bias_sigma=self._main.imu.acc_turn_on_bias_sigma.get(),
                acc_lpf_cutoff_freq=self._main.imu.acc_lpf_cutoff_freq.get(),
            )
            robot.append(imu_model)

            # Magnetometer plugin
            mag_model = MagnetometerModel(
                ns=self._drone_name,
                link_name=root_link,
                update_rate=self._main.imu.update_rate.get(),
                offset=self._main.imu.offset.get(),
                latitude_0=self._main.simulation.latitude_0.get(),
                longitude_0=self._main.simulation.longitude_0.get(),
                altitude_0=self._main.simulation.altitude_0.get(),
                gauss_noise=self._main.imu.mag_gauss_noise.get(),
                uniform_noise=self._main.imu.mag_uniform_noise.get(),
            )
            robot.append(mag_model)

        # Barometer plugin
        if self._main.barometer.equipped():
            bar_model = BarometerModel(
                ns=self._drone_name,
                link_name=root_link,
                update_rate=self._main.barometer.update_rate.get(),
                offset=self._main.barometer.offset.get(),
                altitude_0=self._main.simulation.altitude_0.get(),
                pressure_var=self._main.barometer.pressure_var.get(),
            )
            robot.append(bar_model)

        # GPS plugin
        if self._main.gps.equipped():
            gps_model = GpsModel(
                ns=self._drone_name,
                link_name=root_link,
                offset=self._main.gps.offset.get(),
                update_rate=self._main.gps.update_rate.get(),
                delay=self._main.gps.delay.get(),
                pos_corr_time=self._main.gps.pos_corr_time.get(),
                hor_pos_accuracy=self._main.gps.horizontal_pos_accuracy.get(),
                ver_pos_accuracy=self._main.gps.vertical_pos_accuracy.get(),
                hor_vel_stddev=self._main.gps.horizontal_vel_stddev.get(),
                ver_vel_stddev=self._main.gps.vertical_vel_stddev.get(),
                latitude_0=self._main.simulation.latitude_0.get(),
                longitude_0=self._main.simulation.longitude_0.get(),
                altitude_0=self._main.simulation.altitude_0.get(),
            )
            robot.append(gps_model)

        # RGB Camera plugin
        if self._main.rgb_camera.equipped():
            add_rgb_camera_model(
                robot=robot,
                ns=self._drone_name,
                link_name=self._main.rgb_camera.link.get(),
                offset=Origin(
                    x=self._main.rgb_camera.offset.x(),
                    y=self._main.rgb_camera.offset.y(),
                    z=self._main.rgb_camera.offset.z(),
                    roll=self._main.rgb_camera.offset.roll(),
                    pitch=self._main.rgb_camera.offset.pitch(),
                    yaw=self._main.rgb_camera.offset.yaw(),
                ),
                frame_rate=self._main.rgb_camera.update_rate.get(),
                width=self._main.rgb_camera.image_width.get(),
                height=self._main.rgb_camera.image_height.get(),
                near=self._main.rgb_camera.depth_range.min(),
                far=self._main.rgb_camera.depth_range.max(),
                fov=self._main.rgb_camera.fov.get(),
                noise_mean=0.0,
                noise_stddev=self._main.rgb_camera.noise_stddev.get(),
            )

        # Depth Camera plugin
        if self._main.depth_camera.equipped():
            add_depth_camera_model(
                robot=robot,
                ns=self._drone_name,
                link_name=self._main.depth_camera.link.get(),
                offset=Origin(
                    x=self._main.depth_camera.offset.x(),
                    y=self._main.depth_camera.offset.y(),
                    z=self._main.depth_camera.offset.z(),
                    roll=self._main.depth_camera.offset.roll(),
                    pitch=self._main.depth_camera.offset.pitch(),
                    yaw=self._main.depth_camera.offset.yaw(),
                ),
                frame_rate=self._main.depth_camera.update_rate.get(),
                width=self._main.depth_camera.image_width.get(),
                height=self._main.depth_camera.image_height.get(),
                near=self._main.depth_camera.depth_range.min(),
                far=self._main.depth_camera.depth_range.max(),
                fov=self._main.depth_camera.fov.get(),
                baseline=self._main.depth_camera.baseline.get(),
                noise_model=self._main.depth_camera.noise_model.get(),
            )

        # LiDAR plugin
        if self._main.lidar.equipped():
            add_lidar_model(
                robot=robot,
                ns=self._drone_name,
                link_name=root_link,
                offset=Origin.Trans(*self._main.lidar.offset.get()),
                update_rate=self._main.lidar.update_rate.get(),
                hor_samples=self._main.lidar.hor_samples.get(),
                ver_samples=self._main.lidar.ver_samples.get(),
                hor_fov=self._main.lidar.hor_fov.get(),
                ver_fov=self._main.lidar.ver_fov.get(),
                dist_range=self._main.lidar.range.get(),
                resolution=self._main.lidar.resolution.get(),
                noise_stddev=self._main.lidar.noise_stddev.get(),
            )

        # Odometry plugin
        if self._main.odometry.equipped():
            odometry_model = OdometryModel(
                ns=self._drone_name,
                link_name=root_link,
                update_rate=self._main.odometry.update_rate.get(),
                offset=self._main.odometry.offset.get(),
                pos_normal_noise_std=self._main.odometry.pos_normal_noise_std.get(),
                rot_normal_noise_std=self._main.odometry.rot_normal_noise_std.get(),
                linvel_normal_noise_std=self._main.odometry.linvel_normal_noise_std.get(),
                angvel_normal_noise_std=self._main.odometry.angvel_normal_noise_std.get(),
                pos_uniform_noise_scale=self._main.odometry.pos_uniform_noise_scale.get(),
                rot_uniform_noise_scale=self._main.odometry.rot_uniform_noise_scale.get(),
                linvel_uniform_noise_scale=self._main.odometry.linvel_uniform_noise_scale.get(),
                angvel_uniform_noise_scale=self._main.odometry.angvel_uniform_noise_scale.get(),
            )
            robot.append(odometry_model)

        # Tether Station plugin
        if self._main.tether_station.equipped():
            add_tether_station_model(
                robot=robot,
                link_name=self._main.tether_station.link.get(),
                world_end=self._main.tether_station.world_end.get(),
                drone_end=self._main.tether_station.drone_end.get(),
                tension=self._main.tether_station.tension.get(),
            )

        # Ground Truth State plugin
        state_gt_model = GroundTruthStateModel(self._drone_name, root_link)
        robot.append(state_gt_model)

        # ROS Control
        ros_control = GazeboRosControl(self._drone_name)
        robot.append(ros_control)
