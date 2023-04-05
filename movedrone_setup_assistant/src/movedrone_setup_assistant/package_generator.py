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
from urdf_tools_py.gazebo import GazeboRosControl, Camera
from dh_rqt_tools.path import get_proj_path
from dh_rqt_tools.messages import q_info, q_error

from .utils import *
from .xml_nodes import *


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
        propellers = self._main.settings.propellers.selected
        battery = self._main.settings.battery
        author_info = self._main.settings.author_information
        ros_pkg = self._main.settings.ros_package

        if propellers.num() < 3:
            # TODO: ヘリのような駆動関節の先にプロペラが付いたモデルなら2つでもいけるはず
            q_error(self._main, "[Propellers] At least 3 propellers are needed.")
            return False

        pins_uniq = set(pin.value() for pin in propellers.pins)
        if len(pins_uniq) != propellers.num():
            q_error(self._main, "[Propellers] Duplicated Pin IDs.")
            return False

        C_cont = battery.C_cont.get()
        C_pulse = battery.C_pulse.get()
        if C_cont > C_pulse:
            q_error(self._main, "[Propellers] C_cont cannot be greater than C_pulse.")

        author_name = author_info.name.get()
        if author_name == "":
            q_error(self._main, "[Author Info] Author name is blank.")
            return False

        author_email = author_info.email.get()
        if not is_valid_email(author_email):
            q_error(self._main, "[Author Info] Invalid email address.")
            return False

        pardir = ros_pkg.pkg_path.pardir
        if not osp.isdir(pardir):
            q_error(self._main, f'[ROS Package] {pardir} does not exist.')
            return False

        pkg_name = ros_pkg.pkg_path.pkg_name
        if pkg_name.count("/") > 0 or pkg_name.count(" "):
            q_error(self._main, f'[ROS Package] Invalid package name.')
            return False

        pkg_path = ros_pkg.pkg_path.text()
        if osp.exists(pkg_path):
            q_error(self._main, f'[ROS Package] {pkg_path} already exists.')
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

        # ファイルを作る
        items = self._make_template_items()
        self._generate_from_template(items, "README.md", pkg_path)
        self._generate_from_template(items, "CMakeLists.txt", pkg_path)
        self._generate_from_template(items, "package.xml", pkg_path)
        self._generate_from_template(items, "observer.yaml", config_dir)
        self._generate_from_template(items, "controller.yaml", config_dir)
        self._generate_from_template(items, "gazebo.launch", launch_dir)
        self._generate_from_template(items, "real.launch", launch_dir)
        self._generate_from_template(items, "bringup.launch", launch_dir)
        self._generate_drone_props(config_dir)
        self._generate_joint_control(config_dir)
        self._generate_urdf(urdf_dir)

    def _make_template_items(self) -> None:
        template_items = {}

        template_items["drone_name"] = self._drone_name

        # Ros Package
        ros_pkg = self._main.settings.ros_package
        template_items["pkg_name"] = ros_pkg.pkg_name.get()

        # Author Info
        author_info = self._main.settings.author_information
        template_items["author_name"] = author_info.name.get()
        template_items["author_email"] = author_info.email.get()

        # IMU
        imu = self._main.settings.imu
        template_items["gyro_noise_density"] = imu.gyro_noise_density.get()
        template_items["gyro_random_walk"] = imu.gyro_random_walk.get()
        template_items["acc_noise_density"] = imu.acc_noise_density.get()
        template_items["acc_random_walk"] = imu.acc_random_walk.get()

        # Simulation
        simulation = self._main.settings.simulation
        template_items["ref_mag_north"] = simulation.ref_mag_north.get() * 1e-9
        template_items["ref_mag_east"] = simulation.ref_mag_east.get() * 1e-9
        template_items["ref_mag_down"] = simulation.ref_mag_down.get() * 1e-9

        # Controllers
        controllers = self._main.settings.controllers
        if controllers.controller_type.get() == controllers.LMPC_LABEL:
            lmpc = self._main.settings.controllers.lmpc_settings
            lmpc_items = {
                "natural_freq": lmpc.natural_freq.get(),
                "damp_ratio": lmpc.damp_ratio.get(),
                "pred_horizon": lmpc.pred_horizon.get(),
                "pred_steps": lmpc.pred_steps.get(),
                "rot_decay": lmpc.rot_decay.get(),
                "angvel_decay": lmpc.angvel_decay.get(),
                "rot_weight": lmpc.rot_weight.get(),
                "angvel_weight": lmpc.angvel_weight.get(),
                "thrust_weight": lmpc.thrust_weight.get(),
                "thrust_rate_weight": lmpc.thrust_rate_weight.get(),
            }
            template_items["lmpc"] = lmpc_items
        elif controllers.controller_type.get() == controllers.NMPC_LABEL:
            raise NotImplementedError
        elif controllers.controller_type.get() == controllers.SMC_LABEL:
            raise NotImplementedError
        else:
            raise NotImplementedError

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

    def _generate_drone_props(self, config_dir: str) -> None:
        # yamlファイルに書き込むための辞書を作る
        propellers = self._main.settings.propellers.selected
        battery = self._main.settings.battery
        num_rotors = propellers.num()
        drone_props = {
            "drone_name": self._drone_name,
            "num_rotors": num_rotors,
            "battery_voltage": battery.voltage.get(),
            "required_joint_names": self._main.urdf_parser.required_joint_names(),
        }
        for i in range(num_rotors):
            drone_props[f'rotor_{i}'] = {
                "link_name": propellers.link_names[i].text(),
                "direction": propellers.directions[i].currentText().lower(),
                "motor_constant": propellers.motor_consts[i].value(),
                "moment_constant": propellers.moment_consts[i].value(),
                "kv": propellers.kvs[i].value(),
                "efficiency": propellers.efficiencies[i].value() * 1e-2,
                "rotor_drag_coefficient": propellers.drag_coefs[i].value(),
                "rolling_moment_coefficient": propellers.rolling_coefs[i].value(),
                "time_constant_up": propellers.time_consts_up[i].value() * 1e-3,
                "time_constant_down": propellers.time_consts_down[i].value() * 1e-3,
                "pin": propellers.pins[i].value(),
            }

        # yamlファイルを作成
        drone_props_path = osp.join(config_dir, "drone_properties.yaml")
        with open(drone_props_path, "w") as f:
            yaml.dump(drone_props, f)

    def _generate_joint_control(self, config_dir: str) -> None:
        # yamlファイルに書き込むための辞書を作る
        jnt_ctrl_sub = {}

        jnt_ctrl_sub["joint_state_controller"] = {
            "type": "joint_state_controller/JointStateController",
            "publish_rate": 1000.
        }

        for jnt_name in self._main.urdf_parser.required_joint_names():
            jnt_ctrl_sub[f'{jnt_name}_controller'] = {
                "type": "position_controllers/JointPositionController",
                "joint": jnt_name,
            }

        jnt_ctrl = {self._drone_name: jnt_ctrl_sub}

        # yamlファイルを作成
        jnt_ctrl_path = osp.join(config_dir, "joint_control.yaml")
        with open(jnt_ctrl_path, "w") as f:
            yaml.dump(jnt_ctrl, f)

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
        text += "\nThis may interfere with components automatically added by MoveDrone."
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

        propellers = self._main.settings.propellers.selected
        battery = self._main.settings.battery
        imu = self._main.settings.imu
        magnetometer = self._main.settings.magnetometer
        barometer = self._main.settings.barometer
        gps = self._main.settings.gps
        depth_camera = self._main.settings.depth_camera
        simulation = self._main.settings.simulation

        # Motors
        voltage = battery.voltage.get()
        for i in range(propellers.num()):
            kv = propellers.kvs[i].value()
            efficiency = propellers.efficiencies[i].value() * 1e-2
            max_rot_vel = rpm_to_rad_per_sec(voltage * kv * efficiency)

            motor_model = MotorModel(
                ns=self._drone_name,
                motor_number=i,
                link_name=propellers.link_names[i].text(),
                joint_name=propellers.joint_names[i].text(),
                direction=propellers.directions[i].currentText().lower(),
                max_rot_vel=max_rot_vel,
                motor_const=propellers.motor_consts[i].value(),
                moment_const=propellers.moment_consts[i].value(),
                drag_coef=propellers.drag_coefs[i].value(),
                roll_coef=propellers.rolling_coefs[i].value(),
                time_const_up=propellers.time_consts_up[i].value() * 1e-3,
                time_const_down=propellers.time_consts_down[i].value() * 1e-3,
            )
            robot.append(motor_model)

        # IMU
        imu_model = ImuModel(
            ns=self._drone_name,
            link_name=imu.link.get(),
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
                camera=Camera(
                    width=depth_camera.image_width.get(),
                    height=depth_camera.image_height.get(),
                    near=depth_camera.depth_range.min(),
                    far=depth_camera.depth_range.max(),
                    horizontal_fov=depth_camera.fov.get(),
                ),
                frame_rate=depth_camera.update_rate.get(),
                noise_model=depth_camera.noise_model.get(),
                horizontal_fov=depth_camera.fov.get(),
                baseline=depth_camera.baseline.get(),
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
