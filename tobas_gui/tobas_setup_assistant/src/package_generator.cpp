#include "tobas_setup_assistant/package_generator.hpp"

#include <urdf_parser/urdf_parser.h>
#include <QDebug>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_gui_common/command.hpp>
#include <tobas_gui_common/package.hpp>
#include <tobas_path_tools/core.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_ros2_tools/path.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_string_tools/core.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_yaml_tools/core.hpp>

#include "tobas_setup_assistant/xml_elements/xml_elements.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sa
{
ProjectGenerator::ProjectGenerator(rclcpp::Node::SharedPtr node, RobotInfo& robot, SettingsWidget* settings)
  : node_(node), robot_(robot), settings_(settings)
{
  const auto pkg_path = fs::path(ament_index_cpp::get_package_share_directory(kPackageName));
  const auto templates_path = pkg_path / "templates";
  meta_env_ = std::make_shared<TemplateGenerator>(templates_path / "meta_package");
  config_env_ = std::make_shared<TemplateGenerator>(templates_path / "config_package");
  user_msg_env_ = std::make_shared<TemplateGenerator>(templates_path / "user_msg_package");
  user_cpp_env_ = std::make_shared<TemplateGenerator>(templates_path / "user_cpp_package");
  user_py_env_ = std::make_shared<TemplateGenerator>(templates_path / "user_py_package");
}

bool ProjectGenerator::generateProject(const fs::path& tbs_path)
{
  // Tobasパッケージを作成
  if (!path::createDirectories(tbs_path)) {
    qt::qErrorBox(settings_, "Failed to create Tobas project path.");
    return false;
  }

  // テンプレート用アイテムを作成
  const auto tpl_data = createTemplateData(tbs_path);

  // メタパッケージを作成
  if (!generateMetaPackage(tbs_path, tpl_data)) {
    return false;
  }

  // 設定パッケージを作成
  if (!generateConfigPackage(tbs_path, tpl_data)) {
    return false;
  }

  // ユーザ用Msgパッケージを作成
  if (!fs::is_directory(common::getProjUserMsgPkgPath(tbs_path))) {
    if (!generateUserMsgPackage(tbs_path, tpl_data)) {
      return false;
    }
  }

  // ユーザ用C++パッケージを作成
  if (!fs::is_directory(common::getProjUserCppPkgPath(tbs_path))) {
    if (!generateUserCppPackage(tbs_path, tpl_data)) {
      return false;
    }
  }

  // ユーザ用Pythonパッケージを作成
  if (!fs::is_directory(common::getProjUserPyPkgPath(tbs_path))) {
    if (!generateUserPyPackage(tbs_path, tpl_data)) {
      return false;
    }
  }

  // バックアップファイルを作成
  if (!generateBackupFiles(tbs_path)) {
    return false;
  }

  return true;
}

std::string ProjectGenerator::flightActionsPackage() const
{
  if (settings_->controller->isCommandCompatible(tobas::rc_command_t::POS_VEL_YAW)) {
    return "tobas_mr_actions";
  }
  else {
    qt::qWarnBox(
      settings_,
      "The functions for takeoff, landing, and autonomous movement "
      "corresponding to the selected controller have not been implemented yet.");
    return "tobas_dummy_pkg";
  }
}

inja::json ProjectGenerator::createTemplateData(const fs::path& tbs_path)
{
  inja::json tpl_data;

  tpl_data["drone_name"] = robot_.robotName();

  // Controller
  tpl_data["controller_pkg"] = settings_->controller->controllerPackage().toStdString();
  tpl_data["controller_plugin"] = settings_->controller->pluginName().toStdString();
  tpl_data["actions_pkg"] = flightActionsPackage();

  // Hardware
  tpl_data["hardware_pkg"] = settings_->hardware->hardwarePackage();

  // Author Info
  tpl_data["author_name"] = settings_->author_info->authorName().toStdString();
  tpl_data["author_email"] = settings_->author_info->authorEmail().toStdString();

  // Ros Package
  tpl_data["meta_pkg_name"] = common::getProjMetaPkgName(tbs_path);
  tpl_data["config_pkg_name"] = common::getProjCfgPkgName(tbs_path);
  tpl_data["user_msg_pkg_name"] = common::getProjUserMsgPkgName(tbs_path);
  tpl_data["user_cpp_pkg_name"] = common::getProjUserCppPkgName(tbs_path);
  tpl_data["user_py_pkg_name"] = common::getProjUserPyPkgName(tbs_path);

  return tpl_data;
}

tobas::Drone ProjectGenerator::createDrone()
{
  tobas::Drone drone;

  // Drone Name
  drone.name = robot_.robotName();

  // Joints
  const auto& joint_config = settings_->joint_config;
  for (int i = 0; i < joint_config->numJoints(); ++i) {
    tobas::JointConfig joint;
    joint.name = joint_config->getJointName(i).toStdString();
    joint.role = joint_config->getRole(i);
    joint.cmd_iface = joint_config->getCommandInterface(i);
    joint.hw_iface = joint_config->getHardwareInterface(i);
    joint.home_pos = joint_config->getHomePosition(i);
    drone.joints[joint.name] = joint;

    switch (joint.hw_iface) {
      case tobas::hw_iface_t::PWM: {
        tobas::PwmConfig pwm;
        pwm.channel = joint_config->getPwmChannel(i);
        pwm.name = joint.name;
        pwm.period_range.lower = joint_config->getPwmMinPeriod(i);
        pwm.period_range.upper = joint_config->getPwmMaxPeriod(i);
        pwm.value_range.lower = joint_config->getPwmMinAngle(i);
        pwm.value_range.upper = joint_config->getPwmMaxAngle(i);
        pwm.reverse = joint_config->getPwmReverse(i);
        TOBAS_CHECK(drone.pwms.insert({ joint.name, pwm }).second);
        break;
      }
      case tobas::hw_iface_t::OTHER: {
        break;
      }
      default: {
        throw;
      }
    }
  }

  // Propulsion System
  switch (settings_->propulsion_system->type()) {
    case tobas::propulsion_system_t::ELECTRIC: {
      const auto eprop_widget =
        qt::qConstPointerCast<propulsion::electric::PropulsionSystemWidget>(settings_->propulsion_system->selected());
      const auto eprop = std::make_shared<tobas::ElectricPropulsionSystemConfig>();

      // Battery
      const auto battery_widget = eprop_widget->battery;
      eprop->battery.nominal_voltage = battery_widget->nominalVoltage();
      eprop->battery.max_voltage = battery_widget->maxVoltage();
      eprop->battery.sag_voltage = battery_widget->sagVoltage();
      eprop->battery.max_current = battery_widget->maxCurrent();

      // Rotors
      const auto units_widget = eprop_widget->units->selected();
      for (int i = 0; i < eprop_widget->numUnits(); ++i) {
        const auto link_name = eprop_widget->linkName(i).toStdString();
        const auto unit_widget = units_widget->widget(i);

        const auto rotor = std::make_shared<tobas::ElectricRotorConfig>();
        rotor->link_name = link_name;
        rotor->direction = unit_widget->general()->direction();
        rotor->axis = robot_.rotorAxisType(link_name);
        rotor->moment_const = unit_widget->aerodynamics()->momentConst();
        rotor->tilt_joint_name = unit_widget->general()->tiltJointName().toStdString();
        rotor->channel = unit_widget->general()->channel();
        rotor->num_poles = unit_widget->motor()->numPoles();
        rotor->kv = unit_widget->motor()->kv();
        rotor->internal_resistance = unit_widget->motor()->internalResistance();
        rotor->propeller_diameter = unit_widget->propeller()->diameter();
        rotor->motor_const = unit_widget->aerodynamics()->motorConst();
        TOBAS_CHECK(eprop->rotors.insert({ link_name, rotor }).second);
      }

      drone.prop = std::static_pointer_cast<tobas::PropulsionSystemConfig>(eprop);
      break;
    }
    case tobas::propulsion_system_t::ICE: {
      const auto iprop_widget =
        qt::qConstPointerCast<propulsion::ice::PropulsionSystemWidget>(settings_->propulsion_system->selected());
      const auto iprop = std::make_shared<tobas::ICEPropulsionSystemConfig>();

      // Engine
      const auto engine_widget = iprop_widget->engine;
      iprop->engine.engine_const = engine_widget->dynamics()->engineConstant();
      iprop->engine.hw_iface = tobas::hw_iface_t::PWM;

      // TODO: PWM以外のインターフェースに対応
      tobas::PwmConfig engine_pwm;
      engine_pwm.channel = engine_widget->hardwareIface()->pwmChannel();
      engine_pwm.name = tobas::pwm::kEngineThrottleKey;
      engine_pwm.period_range.lower = engine_widget->hardwareIface()->pwmPeriodZeroThrot();
      engine_pwm.period_range.upper = engine_widget->hardwareIface()->pwmPeriodFullThrot();
      engine_pwm.value_range.lower = tobas::kMinThrot;
      engine_pwm.value_range.upper = tobas::kMaxThrot;
      engine_pwm.reverse = false;
      TOBAS_CHECK(drone.pwms.insert({ tobas::pwm::kEngineThrottleKey, engine_pwm }).second);

      // Rotors
      const auto units_widget = iprop_widget->units->selected();
      for (int i = 0; i < iprop_widget->numUnits(); ++i) {
        const auto link_name = iprop_widget->linkName(i).toStdString();
        const auto unit_widget = units_widget->widget(i);

        const auto rotor = std::make_shared<tobas::ICERotorConfig>();
        rotor->link_name = link_name;
        rotor->direction = unit_widget->general()->direction();
        rotor->axis = robot_.rotorAxisType(link_name);
        rotor->moment_const = unit_widget->aerodynamics()->momentConst();
        rotor->tilt_joint_name = "";
        rotor->gear_ratio = unit_widget->transmission()->gearRatio();
        rotor->pitch_ref = unit_widget->propeller()->pitchAngleRef();
        rotor->pitch_limit = unit_widget->propeller()->pitchAngleLimit();
        rotor->motor_const = unit_widget->aerodynamics()->motorConst();
        rotor->hw_iface = tobas::hw_iface_t::PWM;
        TOBAS_CHECK(iprop->rotors.insert({ link_name, rotor }).second);

        // TODO: PWM以外のインターフェースに対応
        tobas::PwmConfig pitch_pwm;
        pitch_pwm.channel = unit_widget->hardwareIface()->pwmChannel();
        pitch_pwm.name = link_name;
        pitch_pwm.period_range.lower = unit_widget->hardwareIface()->pwmPeriodMinPitch();
        pitch_pwm.period_range.upper = unit_widget->hardwareIface()->pwmPeriodMaxPitch();
        pitch_pwm.value_range = unit_widget->propeller()->pitchAngleLimit();
        pitch_pwm.reverse = false;
        TOBAS_CHECK(drone.pwms.insert({ link_name, pitch_pwm }).second);
      }

      drone.prop = std::static_pointer_cast<tobas::PropulsionSystemConfig>(iprop);
      break;
    }
    default: {
      throw;
    }
  }

  // Fixed Wing
  if (settings_->fixed_wing->hasFixedWing()) {
    drone.fixed_wing = std::make_shared<tobas::FixedWingConfig>();

    // Vehicle
    const auto vehicle = settings_->fixed_wing->vehicle();
    drone.fixed_wing->vehicle.wing_surface = vehicle->wingSurface();
    drone.fixed_wing->vehicle.wing_span = vehicle->wingSpan();
    drone.fixed_wing->vehicle.mac = vehicle->mac();
    drone.fixed_wing->vehicle.ac.data = vehicle->aerodynamicCenter();
    drone.fixed_wing->vehicle.alpha_limit = vehicle->alphaLimit();

    // Aerodynamic Coefficients
    const auto aero_coefs = settings_->fixed_wing->aeroCoefs();
    drone.fixed_wing->aerodynamics.c_lift_0 = aero_coefs->c_lift_0();
    drone.fixed_wing->aerodynamics.c_lift_alpha = aero_coefs->c_lift_alpha();
    drone.fixed_wing->aerodynamics.c_drag_0 = aero_coefs->c_drag_0();
    drone.fixed_wing->aerodynamics.c_drag_alpha = aero_coefs->c_drag_alpha();
    drone.fixed_wing->aerodynamics.c_side_beta = aero_coefs->c_side_beta();
    drone.fixed_wing->aerodynamics.c_roll_beta = aero_coefs->c_roll_beta();
    drone.fixed_wing->aerodynamics.c_roll_p = aero_coefs->c_roll_p();
    drone.fixed_wing->aerodynamics.c_roll_r = aero_coefs->c_roll_r();
    drone.fixed_wing->aerodynamics.c_pitch_0 = aero_coefs->c_pitch_0();
    drone.fixed_wing->aerodynamics.c_pitch_alpha = aero_coefs->c_pitch_alpha();
    drone.fixed_wing->aerodynamics.c_pitch_abs_beta = aero_coefs->c_pitch_abs_beta();
    drone.fixed_wing->aerodynamics.c_pitch_alpha_rate = aero_coefs->c_pitch_alpha_rate();
    drone.fixed_wing->aerodynamics.c_pitch_q = aero_coefs->c_pitch_q();
    drone.fixed_wing->aerodynamics.c_yaw_beta = aero_coefs->c_yaw_beta();
    drone.fixed_wing->aerodynamics.c_yaw_p = aero_coefs->c_yaw_p();
    drone.fixed_wing->aerodynamics.c_yaw_r = aero_coefs->c_yaw_r();

    // Control Surfaces
    const auto css = settings_->fixed_wing->controlSurfaces()->selected();
    for (int i = 0; i < css->numUnits(); ++i) {
      const auto link_name = css->linkName(i).toStdString();

      tobas::ControlSurface cs;
      cs.link_name = link_name;
      cs.c_lift_delta = css->liftCoef(i);
      cs.c_drag_abs_delta = css->dragCoef(i);  // TODO: 正負の確認が必要？
      cs.c_side_delta = css->sideCoef(i);
      cs.c_roll_delta = css->rollCoef(i);
      cs.c_pitch_delta = css->pitchCoef(i);
      cs.c_yaw_delta = css->yawCoef(i);

      drone.fixed_wing->control_surfaces[link_name] = cs;
    }
  }

  // RC Input
  drone.num_sbus_channels = settings_->rc_input->numOfSbusChannels();

  return drone;
}

bool ProjectGenerator::hasServoJoint() const
{
  const auto& joint_config = settings_->joint_config;
  for (int i = 0; i < joint_config->numJoints(); ++i) {
    if (tobas::isServoJoint(joint_config->getRole(i))) {
      return true;
    }
  }
  return false;
}

bool ProjectGenerator::generateMetaPackage(const fs::path& tbs_path, const inja::json& tpl_data)
{
  const auto meta_pkg_path = common::getProjMetaPkgPath(tbs_path);
  fs::create_directory(meta_pkg_path);

  meta_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", meta_pkg_path);
  meta_env_->generate(tpl_data, "package.xml.tplxml", meta_pkg_path);

  if (!createEmptyFile(meta_pkg_path / kDoNotEditThisPackage)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateConfigPackage(const fs::path& tbs_path, const inja::json& tpl_data)
{
  const auto pkg_path = common::getProjCfgPkgPath(tbs_path);
  fs::create_directory(pkg_path);

  // ディレクトリを作成
  const auto config_dir = common::getProjCfgConfigDirPath(tbs_path);
  const auto launch_dir = common::getProjCfgLaunchDirPath(tbs_path);
  const auto mesh_dir = common::getProjCfgMeshDirPath(tbs_path);
  const auto urdf_dir = common::getProjCfgUrdfDirPath(tbs_path);
  fs::create_directory(config_dir);
  fs::create_directory(launch_dir);
  fs::create_directory(mesh_dir);
  fs::create_directory(urdf_dir);

  // テンプレートから生成
  config_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", pkg_path);
  config_env_->generate(tpl_data, "package.xml.tplxml", pkg_path);
  config_env_->generate(tpl_data, std::string(tobas::node::kJointStateBroadcaster) + ".yaml.tplyaml", config_dir);
  config_env_->generate(tpl_data, "component_containers_mp.launch.py.tplpy", launch_dir);
  config_env_->generate(tpl_data, "component_containers_sp.launch.py.tplpy", launch_dir);
  config_env_->generate(tpl_data, "common_realtime_component.launch.py.tplpy", launch_dir);
  config_env_->generate(tpl_data, "common_realtime_standalone.launch.py.tplpy", launch_dir);
  config_env_->generate(tpl_data, "common_interface.launch.py.tplpy", launch_dir);
  config_env_->generate(tpl_data, "common.launch.py.tplpy", launch_dir);
  config_env_->generate(tpl_data, "real_realtime.launch.py.tplpy", launch_dir);
  config_env_->generate(tpl_data, "real_interface.launch.py.tplpy", launch_dir);
  config_env_->generate(tpl_data, "real.launch.py.tplpy", launch_dir);
  config_env_->generate(tpl_data, "gazebo.launch.xml.tplxml", launch_dir);
  config_env_->generate(tpl_data, "hitl.launch.py.tplpy", launch_dir);

  // Dynamic parameters
  if (!createEmptyYaml(common::getProjImuFiltDynParamsPath(tbs_path), false)) {
    return false;
  }
  if (!createEmptyYaml(common::getProjObsvDynParamsPath(tbs_path), false)) {
    return false;
  }
  if (!createEmptyYaml(common::getProjCtrlDynParamsPath(tbs_path), false)) {
    return false;
  }
  if (!createEmptyYaml(common::getProjRcTeleopDynParamsPath(tbs_path), false)) {
    return false;
  }

  // その他
  if (!createEmptyFile(pkg_path / kDoNotEditThisPackage)) {
    return false;
  }
  if (!generateControllerManagerLaunch(tbs_path)) {
    return false;
  }
  if (!generateJointControllerManagerConfig(tbs_path)) {
    return false;
  }
  if (!generateJointControllerConfigs(tbs_path)) {
    return false;
  }
  if (!generateDroneConfig(tbs_path)) {
    return false;
  }
  if (!generatePreArmCheckConfig(tbs_path)) {
    return false;
  }
  if (!generateObserverStaticConfig(tbs_path)) {
    return false;
  }
  if (!generateControllerStaticConfig(tbs_path)) {
    return false;
  }
  if (!generateRcTeleopStaticConfig(tbs_path)) {
    return false;
  }
  if (!generateModifiedUrdf(tbs_path)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateUserMsgPackage(const fs::path& tbs_path, const inja::json& tpl_data)
{
  const auto pkg_path = common::getProjUserMsgPkgPath(tbs_path);

  // パッケージを作成
  fs::create_directory(pkg_path);

  // テンプレートから作成
  user_msg_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", pkg_path, false);
  user_msg_env_->generate(tpl_data, "package.xml.tplxml", pkg_path, false);

  // その他
  if (!createEmptyFile(pkg_path / kYouCanEditThisPackage)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateUserCppPackage(const fs::path& tbs_path, const inja::json& tpl_data)
{
  const auto pkg_path = common::getProjUserCppPkgPath(tbs_path);

  // パッケージを作成
  fs::create_directory(pkg_path);

  // ディレクトリを作成
  const auto launch_dir = pkg_path / "launch";
  const auto nodes_dir = pkg_path / "nodes";
  fs::create_directory(launch_dir);
  fs::create_directory(nodes_dir);

  // テンプレートから作成
  user_cpp_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", pkg_path, false);
  user_cpp_env_->generate(tpl_data, "package.xml.tplxml", pkg_path, false);
  user_cpp_env_->generate(tpl_data, "common_realtime.launch.py.tplpy", launch_dir, false);
  user_cpp_env_->generate(tpl_data, "common_interface.launch.py.tplpy", launch_dir, false);
  user_cpp_env_->generate(tpl_data, "real_realtime.launch.py.tplpy", launch_dir, false);
  user_cpp_env_->generate(tpl_data, "real_interface.launch.py.tplpy", launch_dir, false);
  user_cpp_env_->generate(tpl_data, "gazebo.launch.py.tplpy", launch_dir, false);
  user_cpp_env_->generate(tpl_data, "user_node.cpp.tplcpp", nodes_dir, false);

  // その他
  if (!createEmptyFile(pkg_path / kYouCanEditThisPackage)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateUserPyPackage(const fs::path& tbs_path, const inja::json& tpl_data)
{
  const auto pkg_path = common::getProjUserPyPkgPath(tbs_path);
  const auto pkg_name = common::getProjUserPyPkgName(tbs_path);

  // パッケージを作成
  fs::create_directory(pkg_path);

  // 空のresourceファイルを作成
  const auto resource_file = pkg_path / "resource" / pkg_name;
  path::createFilePath(resource_file);

  // ディレクトリを作成
  const auto launch_dir = pkg_path / "launch";
  const auto lib_dir = pkg_path / pkg_name;
  fs::create_directory(launch_dir);
  fs::create_directory(lib_dir);

  // テンプレートから作成
  user_py_env_->generate(tpl_data, "package.xml.tplxml", pkg_path, false);
  user_py_env_->generate(tpl_data, "setup.cfg.tplini", pkg_path, false);
  user_py_env_->generate(tpl_data, "setup.py.tplpy", pkg_path, false);
  user_py_env_->generate(tpl_data, "common_realtime.launch.py.tplpy", launch_dir, false);
  user_py_env_->generate(tpl_data, "common_interface.launch.py.tplpy", launch_dir, false);
  user_py_env_->generate(tpl_data, "real_realtime.launch.py.tplpy", launch_dir, false);
  user_py_env_->generate(tpl_data, "real_interface.launch.py.tplpy", launch_dir, false);
  user_py_env_->generate(tpl_data, "gazebo.launch.py.tplpy", launch_dir, false);
  user_py_env_->generate(tpl_data, "user_node.py.tplpy", lib_dir, false);

  // その他
  if (!createEmptyFile(pkg_path / kYouCanEditThisPackage)) {
    return false;
  }
  if (!createEmptyFile(lib_dir / "__init__.py")) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateBackupFiles(const fs::path& tbs_path)
{
  // ディレクトリを作成
  const auto backup_dir = common::getProjBackupDirPath(tbs_path);
  fs::create_directory(backup_dir);

  // 設定ファイル
  const auto backup_data = settings_->dump();
  if (!saveYamlNode(common::getProjBackupSettingsPath(tbs_path), backup_data)) {
    return false;
  }

  // Save original URDF
  const auto doc = urdf::exportURDF(*robot_.urdf());
  const auto robot = doc->RootElement();
  if (!replaceOriginalUrdfMeshFilePaths(robot, tbs_path)) {
    return false;
  }
  if (doc->SaveFile(common::getProjBackupUrdfPath(tbs_path).c_str()) != tinyxml2::XML_SUCCESS) {
    qt::qErrorBox(settings_, "Failed to save the original URDF.");
    return false;
  }

  return true;
}

bool ProjectGenerator::generateControllerManagerLaunch(const fs::path& tbs_path)
{
  const auto& ns = robot_.robotName();
  const auto& joint_config = settings_->joint_config;

  // XMLを作成
  const auto doc = new tinyxml2::XMLDocument();
  const auto launch = doc->NewElement("launch");
  doc->InsertFirstChild(launch);

  // サーボジョイントが少なくとも1つ登録されている場合に限りcontroller_managerを立ち上げる
  if (hasServoJoint()) {
    const auto cfg_pkg_name = common::getProjCfgPkgName(tbs_path);
    const auto config_dir = "$(find-pkg-share " + cfg_pkg_name + ")/config/";

    // Joint state broadcaster
    const auto jsb_name = std::string(tobas::node::kJointStateBroadcaster);
    const auto jsb_param = config_dir + jsb_name + ".yaml";
    const auto jsb_args = jsb_name + " --param-file " + jsb_param;
    const auto jsb_node = xml::addNode(launch, "controller_manager", "spawner", "", ns, "", jsb_args);
    xml::addNodeParam(jsb_node, "use_sim_time", "true");

    // コントローラごとにノードを立ち上げる
    for (int i = 0; i < joint_config->numJoints(); ++i) {
      if (!tobas::isServoJoint(joint_config->getRole(i))) {
        continue;
      }

      const auto joint_name = joint_config->getJointName(i).toStdString();
      const auto ctrl_name = joint_name + "_controller";
      const auto ctrl_param = config_dir + ctrl_name + ".yaml";
      const auto ctrl_args = ctrl_name + " --param-file " + ctrl_param;
      const auto ctrl_node = xml::addNode(launch, "controller_manager", "spawner", "", ns, "", ctrl_args);
      xml::addNodeParam(ctrl_node, "use_sim_time", "true");
    }
  }

  // XMLを保存
  const auto launch_dir = common::getProjCfgLaunchDirPath(tbs_path);
  if (doc->SaveFile((launch_dir / "joint_controller_manager.launch.xml").c_str()) != tinyxml2::XML_SUCCESS) {
    qt::qErrorBox(settings_, "Failed to save the controller manager configurations.");
    return false;
  }

  return true;
}

bool ProjectGenerator::generateJointControllerManagerConfig(const fs::path& tbs_path)
{
  // Controller manager
  YAML::Node manager_params_node(YAML::NodeType::Map);
  manager_params_node["update_rate"] = 100;  // TODO: GUIで設定できるように
  manager_params_node[tobas::node::kJointStateBroadcaster]["type"] = tobas::ctrl_manager::type::kJointStateBroadcaster;

  // Each joint controllers
  const auto joint_config = settings_->joint_config;
  for (int i = 0; i < joint_config->numJoints(); ++i) {
    if (!tobas::isServoJoint(joint_config->getRole(i))) {
      continue;
    }

    const auto jnt_name = joint_config->getJointName(i).toStdString();
    const auto ctrl_name = jnt_name + "_controller";
    manager_params_node[ctrl_name]["type"] = tobas::ctrl_manager::type::kForwardCommandController;
  }

  // Create data
  YAML::Node root_node(YAML::NodeType::Map);
  root_node[robot_.robotName()]["controller_manager"][kROSParamsKey] = manager_params_node;

  // Save data
  const auto config_dir = common::getProjCfgConfigDirPath(tbs_path);
  if (!saveYamlNode(config_dir / "joint_controller_manager.yaml", root_node)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateJointControllerConfigs(const fs::path& tbs_path)
{
  const auto joint_config = settings_->joint_config;

  for (int i = 0; i < joint_config->numJoints(); ++i) {
    if (!tobas::isServoJoint(joint_config->getRole(i))) {
      continue;
    }

    const auto jnt_name = joint_config->getJointName(i).toStdString();
    const auto ctrl_name = jnt_name + "_controller";

    YAML::Node ctrl_params_node(YAML::NodeType::Map);
    ctrl_params_node["joints"].push_back(jnt_name);
    ctrl_params_node["interface_name"] = tobas::textFromEnum(joint_config->getCommandInterface(i));

    // Create data
    YAML::Node root_node(YAML::NodeType::Map);
    root_node["/**"][ctrl_name][kROSParamsKey] = ctrl_params_node;  // XXX: 名前空間を指定すると読み込みに失敗する

    // Save data
    const auto config_dir = common::getProjCfgConfigDirPath(tbs_path);
    if (!saveYamlNode(config_dir / (ctrl_name + ".yaml"), root_node)) {
      return false;
    }
  }

  return true;
}

bool ProjectGenerator::generateDroneConfig(const fs::path& tbs_path)
{
  const auto drone = createDrone();

  const auto tbsdrn_path = common::getProjTbsDrnPath(tbs_path);
  if (!drone.save(tbsdrn_path)) {
    qt::qErrorBox(settings_, "Failed to save drone configuration.");
    return false;
  }

  return true;
}

bool ProjectGenerator::generatePreArmCheckConfig(const fs::path& tbs_path)
{
  YAML::Node node(YAML::NodeType::Map);
  node["check_node_connection"] = settings_->pre_arm_check->checkNodeConnection();
  node["check_battery_voltage"] = settings_->pre_arm_check->checkBatteryVoltage();
  node["check_cpu_temperature"] = settings_->pre_arm_check->checkCPUTemperature();
  node["check_rotor_communication"] = settings_->pre_arm_check->checkRotorCommunication();
  node["check_attitude_level"] = settings_->pre_arm_check->checkAttitudeLevel();
  node["check_position_stability"] = settings_->pre_arm_check->checkPositionStability();
  node["check_position_accuracy"] = settings_->pre_arm_check->checkPositionAccuracy();
  node["check_velocity_accuracy"] = settings_->pre_arm_check->checkVelocityAccuracy();
  node["check_attitude_accuracy"] = settings_->pre_arm_check->checkAttitudeAccuracy();
  node["check_heading_accuracy"] = settings_->pre_arm_check->checkHeadingAccuracy();

  const auto config_dir = common::getProjCfgConfigDirPath(tbs_path);
  if (!saveYamlNode(config_dir / "pre_arm_check.yaml", node)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateObserverStaticConfig(const fs::path& tbs_path)
{
  YAML::Node params(YAML::NodeType::Map);
  params["frame_id"] = robot_.tree().getRootName();
  params["position_source"] = "gnss";  // TODO: 選択できるようにする
  params["adaptive_gnss_noise"] = settings_->observer->adaptiveGnssNoise();
  params["adaptive_grav_noise"] = settings_->observer->adaptiveGravityNoise();
  params["do_acc_bias_estimation"] = settings_->observer->doAccelBiasEstimation();
  params["do_gyro_bias_estimation"] = settings_->observer->doGyroBiasEstimation();
  params["do_mag_hard_bias_estimation"] = settings_->observer->doMagHardBiasEstimation();
  params["do_mag_soft_bias_estimation"] = settings_->observer->doMagSoftBiasEstimation();
  params["do_gravity_estimation"] = settings_->observer->doGravityEstimation();
  params["imu_offset"] = Eigen::Vector3d::Zero().eval();        // TODO
  params["barometer_offset"] = Eigen::Vector3d::Zero().eval();  // TODO
  params["gnss_offset"] = Eigen::Vector3d::Zero().eval();       // TODO

  const auto config_dir = common::getProjCfgConfigDirPath(tbs_path);

  // For component
  const auto node_component = params;
  if (!saveYamlNode(config_dir / "observer_static.yaml", node_component)) {
    return false;
  }

  // For standalone
  YAML::Node node_standalone(YAML::NodeType::Map);
  node_standalone["/**"][tobas::node::kObserver][kROSParamsKey] = params;
  if (!saveYamlNode(config_dir / "observer_static_standalone.yaml", node_standalone)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateControllerStaticConfig(const fs::path& tbs_path)
{
  const auto params = settings_->controller->staticParams();
  TOBAS_CHECK(params.IsMap());

  const auto config_dir = common::getProjCfgConfigDirPath(tbs_path);

  // For component
  const auto node_component = params;
  if (!saveYamlNode(config_dir / "controller_static.yaml", node_component)) {
    return false;
  }

  // For standalone
  YAML::Node node_standalone(YAML::NodeType::Map);
  node_standalone["/**"][tobas::node::kController][kROSParamsKey] = params;
  if (!saveYamlNode(config_dir / "controller_static_standalone.yaml", node_standalone)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateRcTeleopStaticConfig(const fs::path& tbs_path)
{
  YAML::Node params(YAML::NodeType::Map);
  params["acrobat_mode"] = settings_->controller->acrobatModeCommand();
  params["stabilize_mode"] = settings_->controller->stabilizeModeCommand();
  params["loiter_mode"] = settings_->controller->loiterModeCommand();

  const auto config_dir = common::getProjCfgConfigDirPath(tbs_path);

  // For component
  const auto node_component = params;
  if (!saveYamlNode(config_dir / "rc_teleop_static.yaml", node_component)) {
    return false;
  }

  // For standalone
  YAML::Node node_standalone(YAML::NodeType::Map);
  node_standalone["/**"][tobas::node::kRcTeleop][kROSParamsKey] = params;
  if (!saveYamlNode(config_dir / "rc_teleop_static_standalone.yaml", node_standalone)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateModifiedUrdf(const fs::path& tbs_path)
{
  // Export the original URDF
  const auto doc = urdf::exportURDF(*robot_.urdf());
  const auto robot = doc->RootElement();

  // Modify robot
  if (!resolveModifiedUrdfMeshFilePaths(robot, tbs_path)) {
    return false;
  }
  if (!removePropellerJointLimits(robot)) {
    return false;
  }
  if (!addXmlElements(robot, tbs_path)) {
    return false;
  }

  // Save modified URDF
  if (doc->SaveFile(common::getProjXacroPath(tbs_path).c_str()) != tinyxml2::XML_SUCCESS) {
    qt::qErrorBox(settings_, "Failed to save the modified URDF.");
    return false;
  }

  return true;
}

bool ProjectGenerator::createEmptyFile(const fs::path& file_path)
{
  if (!path::createFilePath(file_path)) {
    qt::qErrorBox(settings_, "Failed to create \"" + QString::fromStdString(file_path) + "\".");
    return false;
  }

  return true;
}

bool ProjectGenerator::createEmptyYaml(const fs::path& file_path, bool overwrite)
{
  if (!overwrite && fs::is_regular_file(file_path)) {
    return true;
  }

  if (!saveYamlNode(file_path, YAML::Node(YAML::NodeType::Map))) {
    return false;
  }

  return true;
}

bool ProjectGenerator::saveYamlNode(const fs::path& path, const YAML::Node& node)
{
  if (!yaml::save(path, node)) {
    qt::qErrorBox(settings_, "Failed to save \"" + QString::fromStdString(path) + "\".");
    return false;
  }

  return true;
}

bool ProjectGenerator::resolveModifiedUrdfMeshFilePaths(tinyxml2::XMLElement* elem, const fs::path& tbs_path)
{
  if (strcmp(elem->Name(), "mesh") == 0) {
    const auto filename = elem->Attribute("filename");
    if (!filename) {
      qt::qErrorBox(settings_, "Mesh element does not have attribute: \"filename\"");
      return false;
    }

    const auto src_path = ros2::resolveURI(filename);
    if (!fs::exists(src_path)) {
      qt::qErrorBox(settings_, "Mesh file " + QString::fromStdString(src_path) + " does not exist.");
      return false;
    }

    // 必要に応じてメッシュファイルをTobasパッケージ以下にコピー
    const auto mesh_dir = common::getProjCfgMeshDirPath(tbs_path);
    const auto base_name = src_path.filename();
    const auto dst_path = mesh_dir / base_name;
    if (fs::exists(dst_path)) {
      // dst_pathが存在するがsrc_pathと内容が異なる場合は，fs::copy_fileでは上書きされないため一度削除した上でコピーする．
      if (!fs::equivalent(src_path, dst_path)) {
        if (!fs::remove(dst_path)) {
          qt::qErrorBox(settings_, "Failed to remove " + QString::fromStdString(dst_path) + ".");
          return false;
        }

        if (!fs::copy_file(src_path, dst_path)) {
          qt::qErrorBox(
            settings_,
            "Failed to copy " + QString::fromStdString(src_path) + " to " + QString::fromStdString(dst_path) + ".");
          return false;
        }
      }
    }
    else {
      // dst_pathが存在しない場合は，ただコピーすればよい．
      if (!fs::copy_file(src_path, dst_path)) {
        qt::qErrorBox(
          settings_,
          "Failed to copy " + QString::fromStdString(src_path) + " to " + QString::fromStdString(dst_path) + ".");
        return false;
      }
    }

    // メッシュファイルへのパスを置換
    // package://<pkg_name>の書式だとIgnitionが発見できないため，絶対パスに置換できるようxacroコマンドを埋め込む．
    // cf. https://github.com/moveit/moveit_resources/blob/ros2/panda_description/urdf/panda.urdf.xacro
    const auto cfg_pkg_name = common::getProjCfgPkgName(tbs_path);
    const auto new_filename = "file://$(find " + cfg_pkg_name + ")/meshes/" + base_name.string();
    elem->SetAttribute("filename", new_filename.c_str());
  }

  // 再帰的に子要素もチェック
  for (auto child = elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (!resolveModifiedUrdfMeshFilePaths(child, tbs_path)) {
      return false;
    }
  }

  return true;
}

bool ProjectGenerator::replaceOriginalUrdfMeshFilePaths(tinyxml2::XMLElement* elem, const fs::path& tbs_path)
{
  if (strcmp(elem->Name(), "mesh") == 0) {
    const auto filename = elem->Attribute("filename");
    if (!filename) {
      qt::qErrorBox(settings_, "Mesh element does not have attribute: \"filename\"");
      return false;
    }

    const auto src_path = ros2::resolveURI(filename);
    const auto base_name = src_path.filename().string();
    const auto cfg_pkg_name = common::getProjCfgPkgName(tbs_path);

    // config_pkgからの相対パスで指定
    const auto new_filename = "package://" + cfg_pkg_name + "/meshes/" + base_name;
    elem->SetAttribute("filename", new_filename.c_str());
  }

  // 再帰的に子要素もチェック
  for (auto child = elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (!replaceOriginalUrdfMeshFilePaths(child, tbs_path)) {
      return false;
    }
  }

  return true;
}

bool ProjectGenerator::removePropellerJointLimits(tinyxml2::XMLElement* robot)
{
  std::set<std::string> prop_jnt_names;
  const auto& prop = settings_->propulsion_system;
  for (int i = 0; i < prop->numUnits(); ++i) {
    const auto link_name = prop->linkName(i).toStdString();
    const auto jnt_name = robot_.tree().getSegment(link_name)->second.segment.joint().name;
    prop_jnt_names.insert(jnt_name);
  }

  for (auto child = robot->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (strcmp(child->Name(), "joint") == 0) {
      const auto jnt_name = child->Attribute("name");
      if (!jnt_name) {
        qt::qErrorBox(settings_, "Joint element does not have attribute: \"name\"");
        return false;
      }
      if (prop_jnt_names.contains(jnt_name)) {
        for (auto gchild = child->FirstChildElement(); gchild; gchild = gchild->NextSiblingElement()) {
          if (strcmp(gchild->Name(), "limit") == 0) {
            child->DeleteChild(gchild);
            break;
          }
        }
      }
    }
  }

  return true;
}

bool ProjectGenerator::addXmlElements(tinyxml2::XMLElement* robot, const fs::path& tbs_path)
{
  const auto& ns = robot_.robotName();
  const auto& root_name = robot_.tree().getRootName();
  const auto cfg_pkg_name = common::getProjCfgPkgName(tbs_path);

  const auto& prop = settings_->propulsion_system;
  const auto& fmu = settings_->hardware;
  const auto& sim = settings_->simulation;

  const auto drone = createDrone();

  // Get rotor channels
  std::vector<std::string> rotor_link_names;
  for (const auto& [link_name, _] : drone.prop->rotors) {
    rotor_link_names.push_back(link_name);
  }

  // XML namespace
  robot->SetAttribute("xmlns:xacro", "http://ros.org/wiki/xacro");

  // IMU plugin
  xml::addIMUPlugin(
    robot,
    ns,
    root_name,
    fmu->imuUpdateRate(),
    Eigen::Vector3d::Zero(),  // TODO
    fmu->gyroNoiseDensity(),
    fmu->gyroRandomWalk(),
    fmu->gyroBiasCorrTime(),
    fmu->accNoiseDensity(),
    fmu->accRandomWalk(),
    fmu->accBiasCorrTime(),
    rotor_link_names);

  // Magnetometer plugin
  xml::addMagnetometerPlugin(
    robot,
    ns,
    root_name,
    fmu->magUpdateRate(),
    Eigen::Vector3d::Zero(),  // TODO
    sim->latitudeZero(),
    sim->longitudeZero(),
    sim->altitudeZero(),
    fmu->magNoiseStddev(),
    fmu->magHardBiasNorm());

  // Barometer plugin
  xml::addBarometerPlugin(
    robot,
    ns,
    root_name,
    fmu->presUpdateRate(),
    Eigen::Vector3d::Zero(),  // TODO
    sim->altitudeZero(),
    fmu->presNoiseStddev());

  // GNSS plugin
  xml::addGNSSPlugin(
    robot,
    ns,
    root_name,
    fmu->gnssUpdateRate(),
    Eigen::Vector3d::Zero(),  // TODO
    0.1,
    10.,
    fmu->gnssHorizontalPositionAccuracy(),
    fmu->gnssVerticalPositionAccuracy(),
    fmu->gnssHorizontalVelocityStddev(),
    fmu->gnssVerticalVelocityStddev(),
    sim->latitudeZero(),
    sim->longitudeZero(),
    sim->altitudeZero());

  // Propulsion system plugins
  switch (drone.prop->type()) {
    case tobas::propulsion_system_t::ELECTRIC: {
      const auto eprop = qt::qConstPointerCast<propulsion::electric::PropulsionSystemWidget>(prop->selected());
      const auto battery = eprop->battery;
      const auto units = eprop->units->selected();

      // Battery plugin
      constexpr double kBatterySamplingRate = 100.;  // TODO: サンプリングレートをGUIで設定
      xml::addBatteryPlugin(
        robot,
        ns,
        kBatterySamplingRate,
        battery->maxVoltage(),
        battery->sagVoltage(),
        battery->maxCurrent(),
        battery->capacity(),
        battery->internalRegistance(),
        rotor_link_names);

      // Rotor plugins
      for (int i = 0; i < units->numUnits(); ++i) {
        const auto link_name = eprop->linkName(i).toStdString();

        const auto unit = units->widget(i);
        const auto general = unit->general();
        const auto esc = unit->esc();
        const auto motor = unit->motor();
        const auto propeller = unit->propeller();
        const auto aero = unit->aerodynamics();

        xml::addElectricPropulsionSystemPlugin(
          robot,
          ns,
          link_name,
          motor->kv(),
          motor->internalResistance(),
          propeller->numBlade(),
          aero->motorConst(),
          aero->momentConst(),
          aero->dragConst(),
          general->direction(),
          esc->maxCurrent(),
          sim->maxModelErrorRate());
      }

      break;
    }
    case tobas::propulsion_system_t::ICE: {
      const auto iprop = qt::qConstPointerCast<propulsion::ice::PropulsionSystemWidget>(prop->selected());
      const auto engine = iprop->engine;
      const auto units = iprop->units->selected();

      xml::EngineParam engine_param;
      engine_param.engine_const = engine->dynamics()->engineConstant();
      engine_param.time_const_up = engine->response()->timeConstUp();
      engine_param.time_const_down = engine->response()->timeConstDown();

      std::vector<xml::ICERotorParam> rotor_params;
      for (int i = 0; i < units->numUnits(); ++i) {
        const auto unit = units->widget(i);

        xml::ICERotorParam rotor_param;
        rotor_param.link_name = iprop->linkName(i).toStdString();
        rotor_param.direction = unit->general()->direction();
        rotor_param.gear_ratio = unit->transmission()->gearRatio();
        rotor_param.num_blades = unit->propeller()->numBlade();
        rotor_param.pitch_angle_limit = unit->propeller()->pitchAngleLimit();
        rotor_param.max_pitch_angle_rate = unit->propeller()->maxPitchAngleRate();
        rotor_param.motor_const = unit->aerodynamics()->motorConst();
        rotor_param.moment_const = unit->aerodynamics()->momentConst();
        rotor_param.drag_const = unit->aerodynamics()->dragConst();

        rotor_params.push_back(rotor_param);
      }

      xml::addICEPropulsionSystemPlugin(robot, ns, engine_param, rotor_params);

      break;
    }
    default: {
      throw;
    }
  }

  // Fixed wing plugin
  if (drone.fixed_wing) {
    xml::addFixedWingPlugin(robot, ns, root_name, sim->altitudeZero(), *drone.fixed_wing);
  }

  // Wind plugin
  xml::addGazeboWindPlugin(robot, ns, root_name);

  // Ground truth state plugin
  xml::addGazeboGroundTruthStatePlugin(robot, ns, root_name);

  // LookAt position plugin
  xml::addGazeboLookAtPositionPlugin(robot, ns, root_name);

  // Gazebo ROS2 control system
  xml::addGazeboROS2SimSystem(robot, drone.joints);

  // Gazebo ROS2 control plugin
  // XXX: This must be defined after GazeboSimSystem
  if (hasServoJoint()) {
    xml::addGazeboSimROS2ControlPlugin(robot, ns, cfg_pkg_name, "config/joint_controller_manager.yaml");
  }

  // Base static joint for debug
  xml::addBaseStaticJoint(robot, robot_.tree().getRootName());

  return true;
}
}  // namespace sa
}  // namespace gui
