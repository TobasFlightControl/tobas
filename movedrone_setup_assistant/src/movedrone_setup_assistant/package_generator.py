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

        for min_pwm, max_pwm in zip(propellers.min_pwm_periods, propellers.max_pwm_periods):
            lb = min_pwm.value()
            ub = max_pwm.value()
            if lb > ub:
                q_error(self._main, f'[Propellers] Invalid PWM period range: {lb, ub}')
                return False

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

        # LMPC
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

        # NMPC
        nmpc = self._main.settings.controllers.nmpc_settings
        nmpc_items = {}  # TODO
        template_items["nmpc"] = nmpc_items

        # SMC
        smc = self._main.settings.controllers.smc_settings
        smc_items = {}  # TODO
        template_items["smc"] = smc_items

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
        propellers_widget = self._main.settings.propellers.selected
        num_rotors = propellers_widget.num()
        drone_props = {
            "drone_name": self._drone_name,
            "num_rotors": num_rotors,
            "required_joint_names": self._main.urdf_parser.required_joint_names(),
        }
        for i in range(num_rotors):
            drone_props[f'rotor_{i}'] = {
                "link_name": propellers_widget.link_names[i].text(),
                "direction": propellers_widget.directions[i].currentText().lower(),
                "max_velocity": propellers_widget.max_vels[i].value(),
                "motor_constant": propellers_widget.motor_consts[i].value(),
                "moment_constant": propellers_widget.moment_consts[i].value(),
                "rotor_drag_coefficient": propellers_widget.drag_coefs[i].value(),
                "rolling_moment_coefficient": propellers_widget.rolling_coefs[i].value(),
                "time_constant_up": propellers_widget.time_consts_up[i].value(),
                "time_constant_down": propellers_widget.time_consts_down[i].value(),
                "pin": propellers_widget.pins[i].value(),
                "pwm_period": {
                    "min": propellers_widget.min_pwm_periods[i].value(),
                    "max": propellers_widget.max_pwm_periods[i].value(),
                }
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
        root = ET.fromstring(description)
        assert root.tag == "robot"

        self._screen_xml_elements(root)
        self._add_xml_elements(root)

        return ET.ElementTree(root)

    def _screen_xml_elements(self, root: ET.Element) -> None:
        """ 悪影響を与えるかもしれないXML要素を，ユーザに確認した上で消す． """
        for child in root:
            # transmissionは問答無用で消す
            if child.tag == "transmission":
                root.remove(child)

            # gazeboタグの場合はその子ノードを確認する
            if child.tag == "gazebo":
                for gchild in child:
                    if gchild.tag == "plugin":
                        # RotorSのプラグインは問答無用で消す
                        if gchild.attrib["filename"].startswith("librotors"):
                            root.remove(child)
                            continue
                        # Gazebo ROS Controlは問答無用で消す
                        if gchild.attrib["filename"] == "libgazebo_ros_control.so":
                            root.remove(child)
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

    def _add_xml_elements(self, root: ET.Element) -> None:
        root_link = self._main.urdf_parser.get_root().name

        propellers_widget = self._main.settings.propellers.selected
        imu_widget = self._main.settings.imu
        mag_widget = self._main.settings.magnetometer
        bar_widget = self._main.settings.barometer
        gps_widget = self._main.settings.gps
        sim_widget = self._main.settings.simulation

        # Base
        # base_model = BaseModel(self._drone_name, root_link)
        # root.append(base_model)

        # Motors
        for i in range(propellers_widget.num()):
            motor_model = MotorModel(
                self._drone_name,
                i,
                propellers_widget.link_names[i].text(),
                propellers_widget.joint_names[i].text(),
                propellers_widget.directions[i].currentText().lower(),
                propellers_widget.max_vels[i].value(),
                propellers_widget.motor_consts[i].value(),
                propellers_widget.moment_consts[i].value(),
                propellers_widget.drag_coefs[i].value(),
                propellers_widget.rolling_coefs[i].value(),
                propellers_widget.time_consts_up[i].value(),
                propellers_widget.time_consts_down[i].value(),
            )
            root.append(motor_model)

        # Controller Interface
        # controller_interface = ControllerInterface(self._drone_name)
        # root.append(controller_interface)

        # IMU
        imu_model = ImuModel(
            self._drone_name,
            imu_widget.link.get(),
            imu_widget.gyro_noise_density.get(),
            imu_widget.gyro_random_walk.get(),
            imu_widget.gyro_bias_corr_time.get(),
            imu_widget.gyro_turn_on_bias_sigma.get(),
            imu_widget.acc_noise_density.get(),
            imu_widget.acc_random_walk.get(),
            imu_widget.acc_bias_corr_time.get(),
            imu_widget.acc_turn_on_bias_sigma.get(),
        )
        root.append(imu_model)

        # Magnetometer
        mag_model = MagnetometerModel(
            self._drone_name,
            mag_widget.link.get(),
            sim_widget.ref_mag_north.get() * 1e-9,
            sim_widget.ref_mag_east.get() * 1e-9,
            sim_widget.ref_mag_down.get() * 1e-9,
            mag_widget.gauss_noise.get() * 1e-9,
            mag_widget.uniform_noise.get() * 1e-9,
        )
        root.append(mag_model)

        # Barometer
        bar_model = BarometerModel(
            self._drone_name,
            bar_widget.link.get(),
            sim_widget.altitude_0.get(),
            bar_widget.pressure_var.get(),
        )
        root.append(bar_model)

        # GPS
        gps_model = GpsModel(
            self._drone_name,
            gps_widget.link.get(),
            gps_widget.update_rate.get(),
            gps_widget.horizontal_pos_std.get(),
            gps_widget.vertical_pos_std.get(),
            gps_widget.horizontal_vel_std.get(),
            gps_widget.vertical_vel_std.get(),
            sim_widget.latitude_0.get(),
            sim_widget.longitude_0.get(),
        )
        root.append(gps_model)

        # Ground Truth State
        state_gt_model = GroundTruthStateModel(self._drone_name, root_link)
        root.append(state_gt_model)

        # ROS Control
        ros_control = GazeboRosControlModel(self._drone_name)
        root.append(ros_control)

        # Transmissions
        for jnt_name in self._main.urdf_parser.required_joint_names():
            transmission = TransmissionModel(jnt_name, interface=TransmissionModel.POSITION)
            root.append(transmission)

        return root
