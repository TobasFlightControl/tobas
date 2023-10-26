from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant
    from .setting_widgets.propulsion_system.selected_links import SelectedLinkTabWidget

import os
import os.path as osp
import yaml
import rospy
from typing import List
from xml.etree import ElementTree as ET
from jinja2 import Environment, FileSystemLoader
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from urdf_tools_py.core import *
from urdf_tools_py.gazebo import GazeboRosControl
from dh_rqt_tools.path import get_proj_path
from dh_rqt_tools.messages import q_info
from dh_rqt_tools.xml import prettify_and_save
from kdl_sympy.joint import JointType

from tobas_msgs.msg import *

from .utils import *
from .xml_nodes import *


class PackageGenerator(QObject):
    generated = pyqtSignal()

    def __init__(self, main: SetupAssistant):
        super().__init__()
        self._main = main

        self._proj_path = get_proj_path()
        self._template_env = Environment(
            loader=FileSystemLoader(osp.join(self._proj_path, "templates")),
            trim_blocks=True,
            lstrip_blocks=True,
        )

        self._drone_name = ""

    def define_connections(self) -> None:
        self._main.urdf_parser.robot_model_updated.connect(self._on_robot_model_updated)
        self._main.settings.ros_package.generate_button.clicked.connect(
            self._on_generate_button_clicked
        )

    @pyqtSlot()
    def _on_robot_model_updated(self) -> None:
        self._drone_name = get_drone_name()

    @pyqtSlot()
    def _on_generate_button_clicked(self) -> None:
        if not self._is_valid_config():
            return
        self._generate_pkg()
        q_info(self._main, "Configuration package is generated.")
        self.generated.emit()

    def _is_valid_config(self) -> bool:
        if not self._main.settings.start.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.start)
            return False
        if not self._main.settings.battery.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.battery)
            return False
        if not self._main.settings.propulsion_system.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.propulsion_system)
            return False
        if not self._main.settings.fixed_wing.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.fixed_wing)
            return False
        if not self._main.settings.imu.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.imu)
            return False
        if not self._main.settings.barometer.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.barometer)
            return False
        if not self._main.settings.gps.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.gps)
            return False
        if not self._main.settings.rgb_camera.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.rgb_camera)
            return False
        if not self._main.settings.depth_camera.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.depth_camera)
            return False
        if not self._main.settings.lidar.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.lidar)
            return False
        if not self._main.settings.odometry.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.odometry)
            return False
        if not self._main.settings.rc_transmitter.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.rc_transmitter)
            return False
        if not self._main.settings.controller.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.controller)
            return False
        if not self._main.settings.observer.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.observer)
            return False
        if not self._main.settings.simulation.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.simulation)
            return False
        if not self._main.settings.author_information.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.author_information)
            return False
        if not self._main.settings.ros_package.is_valid():
            self._main.settings.switch_to_tab(self._main.settings.ros_package)
            return False

        return True

    def _generate_pkg(self) -> None:
        # 各ディレクトリのパス
        pkg_path = self._main.settings.ros_package.pkg_path.text()
        config_dir = osp.join(pkg_path, "config")
        launch_dir = osp.join(pkg_path, "launch")
        urdf_dir = osp.join(pkg_path, "urdf")

        # ディレクトリを作る
        os.mkdir(pkg_path)
        os.mkdir(config_dir)
        os.mkdir(launch_dir)
        os.mkdir(urdf_dir)

        # テンプレートから生成
        items = self._make_template_items()
        self._generate_from_template(
            items, "CMakeLists.txt", osp.join(pkg_path, "CMakeLists.txt")
        )
        self._generate_from_template(
            items, "package.xml", osp.join(pkg_path, "package.xml")
        )
        self._generate_from_template(
            items, "environment.yaml", osp.join(config_dir, "environment.yaml")
        )
        self._generate_from_template(
            items,
            "hardware_interfaces.yaml",
            osp.join(config_dir, "hardware_interfaces.yaml"),
        )
        self._generate_from_template(
            items,
            "nodelet_manager.launch",
            osp.join(launch_dir, "nodelet_manager.launch"),
        )
        self._generate_from_template(
            items, "common_params.launch", osp.join(launch_dir, "common_params.launch")
        )
        self._generate_from_template(
            items, "gazebo.launch", osp.join(launch_dir, "gazebo.launch")
        )
        self._generate_from_template(
            items, "real.launch", osp.join(launch_dir, "real.launch")
        )
        self._generate_from_template(
            items, "controller.launch", osp.join(launch_dir, "controller.launch")
        )
        self._generate_from_template(
            items, "observer.launch", osp.join(launch_dir, "observer.launch")
        )
        self._generate_from_template(
            items, "bringup.launch", osp.join(launch_dir, "bringup.launch")
        )
        self._generate_from_template(
            items,
            "hardware_interfaces.launch",
            osp.join(launch_dir, "hardware_interfaces.launch"),
        )
        self._generate_from_template(
            items, "hil.launch", osp.join(launch_dir, "hil.launch")
        )
        self._generate_from_template(
            items, "rc_teleop.launch", osp.join(launch_dir, "rc_teleop.launch")
        )

        command_msgs = self._main.settings.controller.selected().COMMAND_MSGS

        # Keyboard Teleop (コントローラの対応コマンドによって場合分け)
        if (
            PositionYaw.__name__ in command_msgs
            or PosVelAccYaw.__name__ in command_msgs
        ):
            self._generate_from_template(
                items,
                "keyboard_teleop/position_yaw.launch",
                osp.join(launch_dir, "keyboard_teleop.launch"),
            )
        elif SpeedRollDeltaPitch.__name__ in command_msgs:
            self._generate_from_template(
                items,
                "keyboard_teleop/speed_roll_dpitch.launch",
                osp.join(launch_dir, "keyboard_teleop.launch"),
            )

        # GUI Teleop (コントローラの対応コマンドによって場合分け)
        if (
            PositionYaw.__name__ in command_msgs
            or PosVelAccYaw.__name__ in command_msgs
        ):
            self._generate_from_template(
                items,
                "gui_teleop/position_yaw.launch",
                osp.join(launch_dir, "gui_teleop.launch"),
            )

        # Pythonで自動生成
        self._generate_drone_config(config_dir)
        self._generate_joint_control_config(config_dir)
        self._generate_rc_teleop_config(config_dir)
        self._generate_controller_config(config_dir)
        self._generate_observer_config(config_dir)
        self._generate_state_checker_config(config_dir)
        self._generate_urdf(urdf_dir)

    def _make_template_items(self) -> None:
        settings = self._main.settings

        template_items = dict()
        template_items["drone_name"] = self._drone_name

        # Sensors
        template_items["imu_update_rate"] = settings.imu.update_rate.get()
        template_items["bar_update_rate"] = settings.barometer.update_rate.get()

        # Controller
        controller = settings.controller
        template_items["controller_pkg"] = controller.controller_pkg()
        template_items["takeoff_pkg"] = controller.takeoff_pkg()
        template_items["landing_pkg"] = controller.landing_pkg()

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
        joint_controllers = "joint_state_controller"
        for jnt_name in self._posture_defining_joint_names():
            joint_controllers += f" {jnt_name}_controller"
        template_items["joint_controllers"] = joint_controllers

        return template_items

    def _generate_from_template(
        self, items: dict, template_file: str, out_path: str
    ) -> None:
        template = self._template_env.get_template(template_file)
        content = template.render(items)  # テンプレートにdict型で文字を埋め込む
        with open(out_path, "w") as f:
            f.write(content)

    def _generate_drone_config(self, config_dir: str) -> None:
        # TBSFファイルに書き込むための辞書を作る
        drone_config = {
            "imu_offset": self._main.settings.imu.offset.get(),
            "barometer_offset": self._main.settings.barometer.offset.get(),
            "gps_offset": self._main.settings.gps.offset.get(),
            "posture_defining_joint_names": self._posture_defining_joint_names(),
        }

        # Propulsion System
        propulsion_system = self._main.settings.propulsion_system.selected

        num_rotors = propulsion_system.count()
        drone_config["num_rotors"] = num_rotors

        for i in range(num_rotors):
            selected: SelectedLinkTabWidget = propulsion_system.widget(i)

            # yaml.dump()時の文字化けを防ぐためにnp.float64から組み込みのfloatに変換
            drone_config[f"rotor_{i}"] = {
                "link_name": selected.link_name(),
                "axis": selected.axis_type(),
                "direction": selected.motor.direction(),
                "rot_speed_coefs": [float(x) for x in selected.motor.rot_speed_coefs()],
                "time_constant_up": float(selected.motor.time_const_up()),
                "time_constant_down": float(selected.motor.time_const_down()),
                "motor_constant": float(selected.aerodynamics.motor_const()),
                "moment_constant": float(selected.aerodynamics.moment_const()),
                "drag_constant": float(selected.aerodynamics.rotor_drag_coef()),
                "pin": i + 1,
            }

            esc = selected.esc
            esc_type = esc.esc_type.currentText()
            drone_config[f"rotor_{i}"]["esc_type"] = esc_type.lower()

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
                "alpha_limit": {
                    "lower": vehicle.alpha_limit.min(),
                    "upper": vehicle.alpha_limit.max(),
                },
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
                    "angle_limit": {
                        "lower": cs.min_angle,
                        "upper": cs.max_angle,
                    },
                    "max_angle_rate": cs.max_angle_rate,
                    "c_lift_delta": cs.c_lift_delta,
                    "c_drag_abs_delta": cs.c_drag_abs_delta,
                    "c_side_delta": cs.c_side_delta,
                    "c_roll_delta": cs.c_roll_delta,
                    "c_pitch_delta": cs.c_pitch_delta,
                    "c_yaw_delta": cs.c_yaw_delta,
                }

        # TBSFファイルを作成
        drone_config_path = osp.join(config_dir, f"{self._drone_name}.tbsf")
        with open(drone_config_path, "w") as f:
            yaml.dump(drone_config, f)

    def _generate_joint_control_config(self, config_dir: str) -> None:
        # yamlファイルに書き込むための辞書を作る
        items = dict()
        items["joint_state_controller"] = {
            "type": "joint_state_controller/JointStateController",
            "publish_rate": 1000.0,
        }
        for jnt_name in self._posture_defining_joint_names():
            items[f"{jnt_name}_controller"] = {
                "type": "position_controllers/JointPositionController",
                "joint": jnt_name,
            }

        # yamlファイルを作成
        jnt_ctrl_path = osp.join(config_dir, "joint_control.yaml")
        with open(jnt_ctrl_path, "w") as f:
            yaml.dump(items, f)

    def _generate_rc_teleop_config(self, config_dir: str) -> None:
        rc_transmitter = self._main.settings.rc_transmitter
        controller = self._main.settings.controller.selected()

        items = dict()
        items["rc_teleop"] = {
            "dead_zone_rate": rc_transmitter.dead_zone_rate.get() / 100.0,
            "mode_names": controller.flight_modes.mode_names(),
        }

        file_path = osp.join(config_dir, "rc_teleop.yaml")
        with open(file_path, "w") as f:
            yaml.dump(items, f)

    def _generate_controller_config(self, config_dir: str) -> None:
        items = self._main.settings.controller.selected().parameter_dict()
        file_path = osp.join(config_dir, "controller.yaml")
        with open(file_path, "w") as f:
            yaml.dump(items, f)

    def _generate_observer_config(self, config_dir: str) -> None:
        items = self._main.settings.observer.selected().parameter_dict()
        file_path = osp.join(config_dir, "observer.yaml")
        with open(file_path, "w") as f:
            yaml.dump(items, f)

    def _generate_state_checker_config(self, config_dir: str) -> None:
        battery = self._main.settings.battery.selected()

        items = dict()
        items["state_checker"] = {
            "battery_voltage_threshold": battery.voltage_threshold(),
        }

        file_path = osp.join(config_dir, "state_checker.yaml")
        with open(file_path, "w") as f:
            yaml.dump(items, f)

    def _generate_urdf(self, urdf_dir: str) -> None:
        robot = self._make_urdf_with_plugins()
        urdf_path = osp.join(urdf_dir, f"{self._drone_name}.xacro")

        # Save URDF
        # ET.ElementTree(robot).write(urdf_path)
        prettify_and_save(robot, urdf_path)

    def _make_urdf_with_plugins(self) -> ET.Element:
        description = rospy.get_param("/robot_description")
        robot = ET.fromstring(description)
        assert robot.tag == "robot"

        self._screen_xml_elements(robot)
        self._add_xml_elements(robot)

        return robot

    def _screen_xml_elements(self, robot: ET.Element) -> None:
        """悪影響を与えるかもしれないXML要素を，ユーザに確認した上で消す．"""
        for child in robot:
            # transmissionは問答無用で消す
            if child.tag == "transmission":
                robot.remove(child)

            # gazeboタグの場合はその子ノードを確認する
            if child.tag == "gazebo":
                for gchild in child:
                    if gchild.tag == "plugin":
                        # RotorSのプラグインは問答無用で消す
                        if gchild.attrib["filename"].startswith("librotors"):
                            robot.remove(child)
                            continue
                        # Gazebo ROS Controlは問答無用で消す
                        if gchild.attrib["filename"] == "libgazebo_ros_control.so":
                            robot.remove(child)
                            continue
                        self._remove_or_keep_gazebo_child(child, gchild)
                    elif gchild.tag == "sensor":
                        self._remove_or_keep_gazebo_child(child, gchild)

    def _remove_or_keep_gazebo_child(
        self, gazebo: ET.Element, child: ET.Element
    ) -> None:
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
        simulation = self._main.settings.simulation

        # XML namespace
        robot.attrib["xmlns:xacro"] = "http://ros.org/wiki/xacro"

        # Base static joint for debug
        base_fix_joint = BaseStaticJoint(root_link=root_link)
        robot.append(base_fix_joint)

        # Base plugin
        base_plugin = BasePlugin(
            ns=self._drone_name,
            rotor_joint_names=propulsion_system.joint_names(),
        )
        robot.append(base_plugin)

        # Wind plugin
        wind_model = WindModel(
            ns=self._drone_name,
            link_name=root_link,
            mean_wind_speed=simulation.mean_wind_speed.get(),
            const_wind_direction=simulation.const_wind_direction.get(),
        )
        robot.append(wind_model)

        # Battery plugin
        battery_model = BatteryModel(
            ns=self._drone_name,
            nominal_voltage=battery.selected().nominal_voltage(),
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
                time_const_up=selected.motor.time_const_up(),
                time_const_down=selected.motor.time_const_down(),
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
                acc_noise_density=imu.acc_noise_density.get(),
                acc_random_walk=imu.acc_random_walk.get(),
                acc_bias_corr_time=imu.acc_bias_corr_time.get(),
                acc_turn_on_bias_sigma=imu.acc_turn_on_bias_sigma.get(),
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
                update_rate=gps.update_rate.get(),
                delay=gps.delay.get(),
                offset=gps.offset.get(),
                hor_pos_std=gps.horizontal_pos_std.get(),
                ver_pos_std=gps.vertical_pos_std.get(),
                hor_vel_std=gps.horizontal_vel_std.get(),
                ver_vel_std=gps.vertical_vel_std.get(),
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
                offset=Origin.Trans(
                    lidar.offset.x(), lidar.offset.y(), lidar.offset.z()
                ),
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

        # Ground Truth State plugin
        state_gt_model = GroundTruthStateModel(self._drone_name, root_link)
        robot.append(state_gt_model)

        # ROS Control
        ros_control = GazeboRosControl(self._drone_name)
        robot.append(ros_control)

        # Transmissions
        for jnt_name in self._posture_defining_joint_names():
            transmission = Transmission(jnt_name, interface=Transmission.POSITION)
            robot.append(transmission)

    def _posture_defining_joint_names(self) -> List[str]:
        """ロボットの形状を決めるのに必要な関節名のリストを返す．"""
        # 固定翼機の車輪などのTransmissionがシミュレーションに悪影響なので，continuousは含めない
        # TODO: いずれは選択制にする (それでもcontinuousは除くべきかも)
        urdf_parser = self._main.urdf_parser
        revolute_joints = set(urdf_parser.search_joint_type(JointType.REVOLUTE))
        prismatic_joints = set(urdf_parser.search_joint_type(JointType.PRISMATIC))
        mobile_joints = revolute_joints | prismatic_joints

        prop_joints = set(self._main.settings.propulsion_system.selected.joint_names())
        cs_joints = set(
            self._main.settings.fixed_wing.control_surfaces.selected.get_joint_names()
        )

        # set演算では"|"よりも"-"が優先されることに注意
        return list(mobile_joints - prop_joints - cs_joints)
