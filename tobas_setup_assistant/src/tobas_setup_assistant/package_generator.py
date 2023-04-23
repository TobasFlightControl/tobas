from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

import os
import os.path as osp
import yaml
import rospy
from xml.etree import ElementTree as ET
from jinja2 import Environment, FileSystemLoader
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from urdf_tools_py.core import *
from urdf_tools_py.gazebo import GazeboRosControl
from dh_rqt_tools.path import get_proj_path
from dh_rqt_tools.messages import q_info

from .utils import *
from .xml_nodes import *
from .setting_widgets.rotary_wings import SelectedLinkTabWidget


class PackageGenerator(QWidget):

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
            return False
        if not self._main.settings.battery.is_valid():
            return False
        if not self._main.settings.rotary_wings.is_valid():
            return False
        if not self._main.settings.imu.is_valid():
            return False
        if not self._main.settings.magnetometer.is_valid():
            return False
        if not self._main.settings.barometer.is_valid():
            return False
        if not self._main.settings.gps.is_valid():
            return False
        if not self._main.settings.rgb_camera.is_valid():
            return False
        if not self._main.settings.depth_camera.is_valid():
            return False
        if not self._main.settings.lidar.is_valid():
            return False
        if not self._main.settings.controller.is_valid():
            return False
        if not self._main.settings.observer.is_valid():
            return False
        if not self._main.settings.simulation.is_valid():
            return False
        if not self._main.settings.author_information.is_valid():
            return False
        if not self._main.settings.ros_package.is_valid():
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
        self._generate_from_template(items, "README.md", pkg_path)
        self._generate_from_template(items, "CMakeLists.txt", pkg_path)
        self._generate_from_template(items, "package.xml", pkg_path)
        self._generate_from_template(items, "environment.yaml", config_dir)
        self._generate_from_template(items, "gazebo.launch", launch_dir)
        self._generate_from_template(items, "real.launch", launch_dir)
        self._generate_from_template(items, "bringup.launch", launch_dir)

        # Pythonで自動生成
        self._generate_drone_config(config_dir)
        self._generate_joint_control_config(config_dir)
        self._generate_controller_config(config_dir)
        self._generate_observer_config(config_dir)
        self._generate_urdf(urdf_dir)

    def _make_template_items(self) -> None:
        template_items = dict()

        template_items["drone_name"] = self._drone_name

        # Controller
        template_items["controller_pkg"] = self._main.settings.controller.pkg_name()

        # Observer
        template_items["observer_pkg"] = self._main.settings.observer.pkg_name()

        # Simulation
        simulation = self._main.settings.simulation
        template_items["gravity"] = simulation.gravity.get()
        template_items["ref_mag_north"] = simulation.ref_mag_north.get() * 1e-9
        template_items["ref_mag_east"] = simulation.ref_mag_east.get() * 1e-9
        template_items["ref_mag_down"] = simulation.ref_mag_down.get() * 1e-9

        # Author Info
        author_info = self._main.settings.author_information
        template_items["author_name"] = author_info.name.get()
        template_items["author_email"] = author_info.email.get()

        # Ros Package
        ros_pkg = self._main.settings.ros_package
        template_items["pkg_name"] = ros_pkg.pkg_name.get()

        # Joint Controllers
        joint_controllers = "joint_state_controller"
        for jnt_name in self._main.urdf_parser.required_joint_names():
            joint_controllers += f' {jnt_name}_controller'
        template_items["joint_controllers"] = joint_controllers

        return template_items

    def _generate_from_template(self, items: dict, template_file: str, dest: str) -> None:
        template = self._template_env.get_template(template_file)
        content = template.render(items)  # テンプレートにdict型で文字を埋め込む
        file_path = osp.join(dest, template_file)
        with open(file_path, "w") as f:
            f.write(content)

    def _generate_drone_config(self, config_dir: str) -> None:
        # yamlファイルに書き込むための辞書を作る
        rotary_wings = self._main.settings.rotary_wings.selected
        battery = self._main.settings.battery
        num_rotors = rotary_wings.count()
        drone_config = {
            "drone_name": self._drone_name,
            "num_rotors": num_rotors,
            "battery_voltage": battery.voltage.get(),
            "required_joint_names": self._main.urdf_parser.required_joint_names(),
        }
        for i in range(num_rotors):
            selected: SelectedLinkTabWidget = rotary_wings.widget(i)
            drone_config[f'rotor_{i}'] = {
                "link_name": selected.link_name(),
                "direction": selected.motor.direction(),
                "kv": selected.motor.kv(),
                "time_constant_up": selected.motor.time_const_up(),
                "time_constant_down": selected.motor.time_const_down(),
                "motor_constant": selected.aerodynamics.motor_const(),
                "moment_constant": selected.aerodynamics.moment_const(),
                "rotor_drag_coefficient": selected.aerodynamics.rotor_drag_coef(),
                "pin": i + 1,
            }

            esc = selected.esc
            esc_type = esc.esc_type.currentText()
            drone_config[f'rotor_{i}']["esc_type"] = esc_type.lower()
            if esc_type == esc.PWM:
                drone_config[f'rotor_{i}']["pwm"] = {
                    "frequency": esc.pwm.freq.get(),
                    "min_pulse_width": esc.pwm.pulse_width_range.min(),
                    "max_pulse_width": esc.pwm.pulse_width_range.max(),
                }
            elif esc_type == esc.DSHOT:
                raise NotImplementedError
            else:
                raise RuntimeError(f'Unknown ESC type: {esc_type}')

        # yamlファイルを作成
        drone_config_path = osp.join(config_dir, "drone_config.yaml")
        with open(drone_config_path, "w") as f:
            yaml.dump(drone_config, f)

    def _generate_joint_control_config(self, config_dir: str) -> None:
        # yamlファイルに書き込むための辞書を作る
        sub_items = dict()
        sub_items["joint_state_controller"] = {
            "type": "joint_state_controller/JointStateController",
            "publish_rate": 1000.
        }
        for jnt_name in self._main.urdf_parser.required_joint_names():
            sub_items[f'{jnt_name}_controller'] = {
                "type": "position_controllers/JointPositionController",
                "joint": jnt_name,
            }
        items = {self._drone_name: sub_items}

        # yamlファイルを作成
        jnt_ctrl_path = osp.join(config_dir, "joint_control.yaml")
        with open(jnt_ctrl_path, "w") as f:
            yaml.dump(items, f)

    def _generate_controller_config(self, config_dir: str) -> None:
        controller = self._main.settings.controller
        controller_type = controller.get_type()

        items = dict()
        if controller_type == controller.LMPC:
            lmpc = controller.lmpc
            items["tobas_controller"] = {
                "natural_frequency": lmpc.natural_freq.get(),
                "damping_ratio": lmpc.damp_ratio.get(),
                "prediction_horizon": lmpc.pred_horizon.get(),
                "prediction_steps": lmpc.pred_steps.get(),
                "rotation_decay": lmpc.rot_decay.get(),
                "angular_velocity_decay": lmpc.angvel_decay.get(),
                "rotation_weight": lmpc.rot_weight.get(),
                "angular_velocity_weight": lmpc.angvel_weight.get(),
                "thrust_force_weight": lmpc.thrust_weight.get(),
                "thrust_force_rate_weight": lmpc.thrust_rate_weight.get(),
            }
        elif controller_type == controller.NMPC:
            raise NotImplementedError
        elif controller_type == controller.SMC:
            raise NotImplementedError
        else:
            raise RuntimeError(f'Unknown controller type: {controller_type}')

        controller_path = osp.join(config_dir, "controller.yaml")
        with open(controller_path, "w") as f:
            yaml.dump(items, f)

    def _generate_observer_config(self, config_dir: str) -> None:
        observer = self._main.settings.observer
        observer_type = observer.get_type()

        items = dict()
        if observer_type == observer.CASCADE:
            cascade = observer.cascade
            items["orientation_estimator_complement"] = {
                "gain_acc": cascade.gain_acc.get(),
                "gain_mag": cascade.gain_mag.get(),
                "bias_alpha": cascade.bias_alpha.get(),
                "do_bias_estimation": cascade.do_bias_estimation.get(),
                "do_adaptive_gain": cascade.do_adaptive_gain.get(),
            }
            items["state_estimator_cascade"] = {
                "use_barometer": True,
                "use_gps_position": True,
                "use_gps_velocity": True,
                "gravity_variance_exp": cascade.grav_var_exp.get(),
            }
        elif observer_type == observer.ESKF:
            imu = self._main.settings.imu
            items["state_estimator_eskf"] = {
                "gyro_noise_density": imu.gyro_noise_density.get(),
                "gyro_random_walk": imu.gyro_random_walk.get(),
                "acc_noise_density": imu.acc_noise_density.get(),
                "acc_random_walk": imu.acc_random_walk.get(),
            }
        else:
            raise RuntimeError(f'Unknown observer type: {observer_type}')

        observer_path = osp.join(config_dir, "observer.yaml")
        with open(observer_path, "w") as f:
            yaml.dump(items, f)

    def _generate_urdf(self, urdf_dir: str) -> None:
        tree = self._make_urdf_with_plugins()
        urdf_path = osp.join(urdf_dir, f'{self._drone_name}.urdf')
        tree.write(urdf_path)

    def _make_urdf_with_plugins(self) -> ET.ElementTree:
        description = rospy.get_param("/robot_description")
        robot = ET.fromstring(description)
        assert robot.tag == "robot"

        self._screen_xml_elements(robot)
        self._add_xml_elements(robot)

        return ET.ElementTree(robot)

    def _screen_xml_elements(self, robot: ET.Element) -> None:
        """ 悪影響を与えるかもしれないXML要素を，ユーザに確認した上で消す． """
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

    def _remove_or_keep_gazebo_child(self, gazebo: ET.Element, child: ET.Element) -> None:
        """ 属性を確認した上でGazeboの子ノードを削除する． """
        msg_box = QMessageBox(self._main)  # 親を設定しておけば一緒に落とせる

        # テキストの設定
        text = f'Gazebo {child.tag} is detected.\n\n'
        for key, value in child.attrib.items():
            text += f'    {key}: {value}\n'
        text += "\nThis may interfere with components automatically added by Tobas."
        msg_box.setText(text)
        msg_box.setInformativeText(f'Do you remove this {child.tag} or keep it?')

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

        rotary_wings = self._main.settings.rotary_wings.selected
        battery = self._main.settings.battery
        imu = self._main.settings.imu
        magnetometer = self._main.settings.magnetometer
        barometer = self._main.settings.barometer
        gps = self._main.settings.gps
        rgb_camera = self._main.settings.rgb_camera
        depth_camera = self._main.settings.depth_camera
        simulation = self._main.settings.simulation

        # Motors
        voltage = battery.voltage.get()
        for i in range(rotary_wings.count()):
            selected: SelectedLinkTabWidget = rotary_wings.widget(i)

            kv = selected.motor.kv()
            max_rot_vel = rpm_to_rad_per_sec(voltage * kv)

            motor_model = MotorModel(
                ns=self._drone_name,
                motor_number=i,
                link_name=selected.link_name(),
                joint_name=selected.joint_name(),
                direction=selected.motor.direction(),
                max_rot_vel=max_rot_vel,
                motor_const=selected.aerodynamics.motor_const(),
                moment_const=selected.aerodynamics.moment_const(),
                rotor_drag_coef=selected.aerodynamics.rotor_drag_coef(),
                time_const_up=selected.motor.time_const_up(),
                time_const_down=selected.motor.time_const_down(),
            )
            robot.append(motor_model)

        # IMU
        imu_model = ImuModel(
            ns=self._drone_name,
            link_name=imu.link.get(),
            update_rate=barometer.update_rate.get(),
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

        # Magnetometer
        mag_model = MagnetometerModel(
            ns=self._drone_name,
            link_name=magnetometer.link.get(),
            update_rate=barometer.update_rate.get(),
            ref_mag_north=simulation.ref_mag_north.get() * 1e-9,
            ref_mag_east=simulation.ref_mag_east.get() * 1e-9,
            ref_mag_down=simulation.ref_mag_down.get() * 1e-9,
            gauss_noise=magnetometer.gauss_noise.get() * 1e-9,
            uniform_noise=magnetometer.uniform_noise.get() * 1e-9,
        )
        robot.append(mag_model)

        # Barometer
        bar_model = BarometerModel(
            ns=self._drone_name,
            link_name=barometer.link.get(),
            update_rate=barometer.update_rate.get(),
            ref_altitude=simulation.altitude_0.get(),
            pressure_var=barometer.pressure_var.get(),
        )
        robot.append(bar_model)

        # GPS
        gps_model = GpsModel(
            ns=self._drone_name,
            link_name=gps.link.get(),
            update_rate=gps.update_rate.get(),
            hor_pos_std=gps.horizontal_pos_std.get(),
            ver_pos_std=gps.vertical_pos_std.get(),
            hor_vel_std=gps.horizontal_vel_std.get(),
            ver_vel_std=gps.vertical_vel_std.get(),
            latitude_0=simulation.latitude_0.get(),
            longitude_0=simulation.longitude_0.get(),
        )
        robot.append(gps_model)

        # RGB Camera
        if not rgb_camera.no_sensor.isChecked():
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
                noise_mean=0.,
                noise_stddev=rgb_camera.noise_stddev.get(),
            )

        # Depth Camera
        if not depth_camera.no_sensor.isChecked():
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

        # Ground Truth State
        state_gt_model = GroundTruthStateModel(self._drone_name, root_link)
        robot.append(state_gt_model)

        # ROS Control
        ros_control = GazeboRosControl(self._drone_name)
        robot.append(ros_control)

        # Transmissions
        for jnt_name in self._main.urdf_parser.required_joint_names():
            transmission = Transmission(jnt_name, interface=Transmission.POSITION)
            robot.append(transmission)
