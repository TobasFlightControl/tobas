#include <ament_index_cpp/get_package_share_directory.hpp>
#include <urdf_parser/urdf_parser.h>
#include <std_msgs/msg/string.hpp>

#include <QDebug>
#include <QMessageBox>

#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/string.hpp>
#include <tobas_path_tools/core.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_ros2_tools/path.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_gui_common/package.hpp>
#include <tobas_gui_common/command.hpp>

#include "tobas_setup_assistant/package_generator.hpp"
#include "tobas_setup_assistant/xml_elements/xml_elements.hpp"

using namespace std;
namespace fs = filesystem;

namespace gui
{
namespace setup_assistant
{
PackageGenerator::PackageGenerator(rclcpp::Node::SharedPtr node, RobotInfo& robot, SettingsWidget* settings)
  : node_(node), robot_(robot), settings_(settings)
{
  const auto pkg_path = fs::path(ament_index_cpp::get_package_share_directory(kPackageName));
  const auto templates_path = pkg_path / "templates";
  meta_env_ = make_shared<TemplateGenerator>(templates_path / "meta_package");
  config_env_ = make_shared<TemplateGenerator>(templates_path / "config_package");
  user_cpp_env_ = make_shared<TemplateGenerator>(templates_path / "user_cpp_package");
  user_py_env_ = make_shared<TemplateGenerator>(templates_path / "user_py_package");
}

bool PackageGenerator::generatePackage()
{
  // Tobasパッケージを作成
  if (!path::createDirectories(tbsPath()))
  {
    qt::qErrorBox(settings_, "Failed to create Tobas package path.");
    return false;
  }

  // バックアップファイルを作成
  if (!generateBackupFiles())
    return false;

  // テンプレート用アイテムを作成
  const auto tpl_data = createTemplateData();

  // メタパッケージを作成
  if (!generateMetaPackage(tpl_data))
    return false;

  // 設定パッケージを作成
  if (!generateConfigPackage(tpl_data))
    return false;

  // ユーザ用C++パッケージを作成
  if (!generateUserCppPackage(tpl_data))
    return false;

  // ユーザ用Pythonパッケージを作成
  if (!generateUserPyPackage(tpl_data))
    return false;

  return true;
}

string PackageGenerator::tbsPath() const
{
  return settings_->ros_package->tbsPath().toStdString();
}

string PackageGenerator::flightActionsPackage() const
{
  if (settings_->controller->isCommandCompatible(tobas::POS_VEL_ACC_YAW))
  {
    return "tobas_mr_actions";
  }
  else
  {
    qt::qWarnBox(
      settings_, "The functions for takeoff, landing, and autonomous movement "
                 "corresponding to the selected controller have not been implemented yet.");
    return "tobas_dummy_pkg";
  }
}

inja::json PackageGenerator::createTemplateData()
{
  inja::json tpl_data;

  tpl_data["drone_name"] = robot_.robotName();

  // Controller
  tpl_data["controller_pkg"] = settings_->controller->controllerPackage().toStdString();
  tpl_data["controller_plugin"] = settings_->controller->pluginName().toStdString();
  tpl_data["actions_pkg"] = flightActionsPackage();

  // Observer
  tpl_data["observer_pkg"] = settings_->observer->observerPackage().toStdString();
  tpl_data["observer_plugin"] = settings_->observer->pluginName().toStdString();

  // Hardware
  tpl_data["hardware_pkg"] = settings_->hardware->hardwarePackage();

  // Author Info
  tpl_data["author_name"] = settings_->author_info->authorName().toStdString();
  tpl_data["author_email"] = settings_->author_info->authorEmail().toStdString();

  // Ros Package
  tpl_data["meta_pkg_name"] = common::getTBSMetaName(tbsPath());
  tpl_data["config_pkg_name"] = common::getTBSConfigName(tbsPath());
  tpl_data["user_cpp_pkg_name"] = common::getTBSUserCppName(tbsPath());
  tpl_data["user_py_pkg_name"] = common::getTBSUserPyName(tbsPath());

  return tpl_data;
}

tobas::Drone PackageGenerator::createDrone()
{
  tobas::Drone drone;

  // Drone Name
  drone.name = robot_.robotName();

  // Battery
  drone.battery.nominal_voltage = settings_->battery->nominalVoltage();
  drone.battery.max_voltage = settings_->battery->maxVoltage();
  drone.battery.sag_voltage = settings_->battery->sagVoltage();
  drone.battery.max_current = settings_->battery->maxCurrent();

  // Joints
  const auto& joint_config = settings_->joint_config;
  for (int i = 0; i < joint_config->count(); ++i)
  {
    tobas::JointConfig joint;
    joint.name = joint_config->getJointName(i).toStdString();
    joint.role = joint_config->getRole(i);
    joint.cmd_iface = joint_config->getCommandInterface(i);
    joint.hw_iface = joint_config->getHardwareInterface(i);
    joint.home_pos = joint_config->getHomePosition(i);
    drone.joints[joint.name] = joint;

    switch (joint.hw_iface)
    {
      case tobas::jnt_hw_iface_t::PWM:
      {
        tobas::PwmConfig pwm;
        pwm.channel = joint_config->getChannel(i);
        pwm.joint_name = joint.name;
        pwm.min_period = 0;  // TODO
        pwm.max_period = 0;  // TODO
        pwm.min_angle = 0.;  // TODO
        pwm.max_angle = 0.;  // TODO
        pwm.reverse = joint_config->getReverse(i);
        drone.pwms[joint.name] = pwm;
        break;
      }
      case tobas::jnt_hw_iface_t::OTHER:
      {
        break;
      }
      default:
      {
        throw;
      }
    }
  }

  // Rotors
  const auto propulsions = settings_->propulsion_system->selected();
  for (int i = 0; i < propulsions->count(); ++i)
  {
    const auto link_name = propulsions->linkName(i);
    const auto prop_config = propulsions->widget(i);

    tobas::RotorConfig rotor;
    rotor.channel = prop_config->general()->channel();
    rotor.link_name = link_name.toStdString();
    rotor.direction = prop_config->motor()->direction();
    rotor.axis = robot_.rotorAxisType(link_name.toStdString());
    rotor.num_poles = prop_config->motor()->numPoles();
    rotor.kv = prop_config->motor()->kv();
    rotor.internal_resistance = prop_config->motor()->internalResistance();
    rotor.propeller_diameter = prop_config->propeller()->diameter();
    rotor.max_rot_speed = prop_config->speedLimit()->maxRotSpeed();
    rotor.motor_constant = prop_config->aerodynamics()->motorConst();
    rotor.moment_constant = prop_config->aerodynamics()->momentConst();
    rotor.drag_constant = prop_config->aerodynamics()->rotorDragCoef();
    rotor.tilt_joint_name = prop_config->general()->tiltJointName().toStdString();

    drone.rotors[rotor.channel] = rotor;
  }

  // Fixed Wing
  drone.fixed_wing.equipped = settings_->fixed_wing->hasFixedWing();
  if (drone.fixed_wing.equipped)
  {
    // Vehicle
    const auto vehicle = settings_->fixed_wing->vehicle();
    drone.fixed_wing.vehicle.wing_surface = vehicle->wingSurface();
    drone.fixed_wing.vehicle.wing_span = vehicle->wingSpan();
    drone.fixed_wing.vehicle.mac = vehicle->mac();
    drone.fixed_wing.vehicle.ac.data = vehicle->aerodynamicCenter();
    drone.fixed_wing.vehicle.alpha_limit.lower = vehicle->alphaLimit().first;
    drone.fixed_wing.vehicle.alpha_limit.upper = vehicle->alphaLimit().second;

    // Aerodynamic Coefficients
    const auto aero_coefs = settings_->fixed_wing->aeroCoefs();
    drone.fixed_wing.aerodynamics.c_lift_0 = aero_coefs->c_lift_0();
    drone.fixed_wing.aerodynamics.c_lift_alpha = aero_coefs->c_lift_alpha();
    drone.fixed_wing.aerodynamics.c_drag_0 = aero_coefs->c_drag_0();
    drone.fixed_wing.aerodynamics.c_drag_alpha = aero_coefs->c_drag_alpha();
    drone.fixed_wing.aerodynamics.c_side_beta = aero_coefs->c_side_beta();
    drone.fixed_wing.aerodynamics.c_roll_beta = aero_coefs->c_roll_beta();
    drone.fixed_wing.aerodynamics.c_roll_p = aero_coefs->c_roll_p();
    drone.fixed_wing.aerodynamics.c_roll_r = aero_coefs->c_roll_r();
    drone.fixed_wing.aerodynamics.c_pitch_0 = aero_coefs->c_pitch_0();
    drone.fixed_wing.aerodynamics.c_pitch_alpha = aero_coefs->c_pitch_alpha();
    drone.fixed_wing.aerodynamics.c_pitch_abs_beta = aero_coefs->c_pitch_abs_beta();
    drone.fixed_wing.aerodynamics.c_pitch_alpha_rate = aero_coefs->c_pitch_alpha_rate();
    drone.fixed_wing.aerodynamics.c_pitch_q = aero_coefs->c_pitch_q();
    drone.fixed_wing.aerodynamics.c_yaw_beta = aero_coefs->c_yaw_beta();
    drone.fixed_wing.aerodynamics.c_yaw_p = aero_coefs->c_yaw_p();
    drone.fixed_wing.aerodynamics.c_yaw_r = aero_coefs->c_yaw_r();

    // Control Surfaces
    const auto css = settings_->fixed_wing->controlSurfaces()->selected();
    for (int i = 0; i < css->count(); ++i)
    {
      const auto link_name = css->linkName(i);
      const auto joint_idx = joint_config->findLink(link_name);

      tobas::ControlSurface cs;
      cs.channel = joint_config->getChannel(joint_idx);
      cs.link_name = link_name.toStdString();
      cs.c_lift_delta = css->liftCoef(i);
      cs.c_drag_abs_delta = css->dragCoef(i);  // TODO: 正負の確認が必要？
      cs.c_side_delta = css->sideCoef(i);
      cs.c_roll_delta = css->rollCoef(i);
      cs.c_pitch_delta = css->pitchCoef(i);
      cs.c_yaw_delta = css->yawCoef(i);

      drone.fixed_wing.control_surfaces[cs.channel] = cs;
    }
  }

  return drone;
}

bool PackageGenerator::hasServoJoint() const
{
  const auto& joint_config = settings_->joint_config;
  for (int i = 0; i < joint_config->count(); ++i)
    if (tobas::isServoJoint(joint_config->getRole(i)))
      return true;
  return false;
}

bool PackageGenerator::generateBackupFiles()
{
  const auto tbs_path = tbsPath();

  // ディレクトリを作成
  const auto backup_dir = common::getBackupPath(tbs_path);
  fs::create_directory(backup_dir);

  // 設定ファイル
  const auto backup_data = settings_->dump();
  if (!saveYamlNode(common::getSettingsPath(tbs_path), backup_data))
    return false;

  // オリジナルURDF
  const auto doc = urdf::exportURDF(*robot_.urdf());
  if (doc->SaveFile(common::getOriginalURDFPath(tbs_path).c_str()) != tinyxml2::XML_SUCCESS)
  {
    qt::qErrorBox(settings_, "Failed to save the original URDF.");
    return false;
  }

  return true;
}

bool PackageGenerator::generateMetaPackage(const inja::json& tpl_data)
{
  const auto meta_pkg_path = common::getTBSMetaPath(tbsPath());
  fs::create_directory(meta_pkg_path);

  meta_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", meta_pkg_path);
  meta_env_->generate(tpl_data, "package.xml.tplxml", meta_pkg_path);

  if (!createEmptyFile(meta_pkg_path / kDoNotEditThisPackage))
    return false;

  return true;
}

bool PackageGenerator::generateConfigPackage(const inja::json& tpl_data)
{
  const auto pkg_path = common::getTBSConfigPath(tbsPath());
  fs::create_directory(pkg_path);

  // ディレクトリを作成
  const auto config_dir = pkg_path / "config";
  const auto launch_dir = pkg_path / "launch";
  const auto urdf_dir = pkg_path / "urdf";
  const auto mesh_dir = pkg_path / "meshes";
  fs::create_directory(config_dir);
  fs::create_directory(launch_dir);
  fs::create_directory(urdf_dir);
  fs::create_directory(mesh_dir);

  // テンプレートから生成
  config_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", pkg_path);
  config_env_->generate(tpl_data, "package.xml.tplxml", pkg_path);
  config_env_->generate(tpl_data, string(tobas::node::kJointStateBroadcaster) + ".yaml.tplyaml", config_dir);
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
  config_env_->generate(tpl_data, "robot_state_publisher.launch.py.tplpy", launch_dir);
  config_env_->generate(tpl_data, "jointpos_commander_gui.launch.py.tplpy", launch_dir);

  // Optional
  if (
    settings_->controller->isCommandCompatible(tobas::rc_command_t::POS_VEL_ACC_YAW)
    || settings_->controller->isCommandCompatible(tobas::rc_command_t::POSE_TWIST_ACCEL))
    config_env_->generate(tpl_data, "base_pose_commander_gui.launch.py.tplpy", launch_dir);

  // Dynamic parameters
  if (!createEmptyYaml(config_dir / "controller_dynamic.yaml", false))
    return false;
  if (!createEmptyYaml(config_dir / "observer_dynamic.yaml", false))
    return false;

  // その他
  if (!createEmptyFile(pkg_path / kDoNotEditThisPackage))
    return false;
  if (!generateControllerManagerLaunch(launch_dir))
    return false;
  if (!generateJointControllerManagerConfig(config_dir))
    return false;
  if (!generateJointControllerConfigs(config_dir))
    return false;
  if (!generateDroneConfig(config_dir))
    return false;
  if (!generateRCTeleopConfig(config_dir))
    return false;
  if (!generateControllerStaticConfig(config_dir))
    return false;
  if (!generateObserverStaticConfig(config_dir))
    return false;
  if (!generateURDF(mesh_dir))
    return false;

  return true;
}

bool PackageGenerator::generateUserCppPackage(const inja::json& tpl_data)
{
  const auto pkg_path = common::getTBSUserCppPath(tbsPath());
  fs::create_directory(pkg_path);

  // ディレクトリを作成
  const auto launch_dir = pkg_path / "launch";
  const auto nodes_dir = pkg_path / "nodes";
  fs::create_directory(launch_dir);
  fs::create_directory(nodes_dir);

  // テンプレートから作成 (存在する場合は上書きしない)
  user_cpp_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", pkg_path, false);
  user_cpp_env_->generate(tpl_data, "package.xml.tplxml", pkg_path, false);
  user_cpp_env_->generate(tpl_data, "common.launch.py.tplpy", launch_dir, false);
  user_cpp_env_->generate(tpl_data, "gazebo.launch.py.tplpy", launch_dir, false);
  user_cpp_env_->generate(tpl_data, "real.launch.py.tplpy", launch_dir, false);
  user_cpp_env_->generate(tpl_data, "user_node.cpp.tplcpp", nodes_dir, false);

  // その他
  if (!createEmptyFile(pkg_path / kYouCanEditThisPackage))
    return false;

  return true;
}

bool PackageGenerator::generateUserPyPackage(const inja::json& tpl_data)
{
  const auto pkg_path = common::getTBSUserPyPath(tbsPath());
  const auto pkg_name = common::getTBSUserPyName(tbsPath());

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

  // テンプレートから作成 (存在する場合は上書きしない)
  user_py_env_->generate(tpl_data, "package.xml.tplxml", pkg_path, false);
  user_py_env_->generate(tpl_data, "setup.cfg.tplini", pkg_path, false);
  user_py_env_->generate(tpl_data, "setup.py.tplpy", pkg_path, false);
  user_py_env_->generate(tpl_data, "common.launch.py.tplpy", launch_dir, false);
  user_py_env_->generate(tpl_data, "gazebo.launch.py.tplpy", launch_dir, false);
  user_py_env_->generate(tpl_data, "real.launch.py.tplpy", launch_dir, false);
  user_py_env_->generate(tpl_data, "user_node.py.tplpy", lib_dir, false);

  // その他
  if (!createEmptyFile(pkg_path / kYouCanEditThisPackage))
    return false;
  if (!createEmptyFile(lib_dir / "__init__.py"))
    return false;

  return true;
}

bool PackageGenerator::generateControllerManagerLaunch(const fs::path& launch_dir)
{
  const auto& ns = robot_.robotName();
  const auto& joint_config = settings_->joint_config;

  // XMLを作成
  const auto doc = new tinyxml2::XMLDocument();
  const auto launch = doc->NewElement("launch");
  doc->InsertFirstChild(launch);

  // サーボジョイントが少なくとも1つ登録されている場合に限りcontroller_managerを立ち上げる
  if (hasServoJoint())
  {
    const auto config_pkg_name = common::getTBSConfigName(tbsPath());
    const auto config_dir = "$(find-pkg-share " + config_pkg_name + ")/config/";

    // Joint state broadcaster
    const auto jsb_name = string(tobas::node::kJointStateBroadcaster);
    const auto jsb_param = config_dir + jsb_name + ".yaml";
    const auto jsb_args = jsb_name + " --param-file " + jsb_param;
    const auto jsb_node = addNode(launch, "controller_manager", "spawner", "", ns, "", jsb_args);
    addNodeParam(jsb_node, "use_sim_time", "true");

    // コントローラごとにノードを立ち上げる
    for (int i = 0; i < joint_config->count(); ++i)
    {
      if (!tobas::isServoJoint(joint_config->getRole(i)))
        continue;

      const auto joint_name = joint_config->getJointName(i).toStdString();
      const auto ctrl_name = joint_name + "_controller";
      const auto ctrl_param = config_dir + ctrl_name + ".yaml";
      const auto ctrl_args = ctrl_name + " --param-file " + ctrl_param;
      const auto ctrl_node = addNode(launch, "controller_manager", "spawner", "", ns, "", ctrl_args);
      addNodeParam(ctrl_node, "use_sim_time", "true");
    }
  }

  // XMLを保存
  if (doc->SaveFile((launch_dir / "joint_controller_manager.launch.xml").c_str()) != tinyxml2::XML_SUCCESS)
  {
    qt::qErrorBox(settings_, "Failed to save the controller manager configurations.");
    return false;
  }

  return true;
}

bool PackageGenerator::generateJointControllerManagerConfig(const fs::path& config_dir)
{
  // Controller manager
  YAML::Node manager_params_node(YAML::NodeType::Map);
  manager_params_node["update_rate"] = 100;  // TODO: GUIで設定できるように
  manager_params_node[tobas::node::kJointStateBroadcaster]["type"] = tobas::ctrl_manager::type::kJointStateBroadcaster;

  // Each joint controllers
  const auto joint_config = settings_->joint_config;
  for (int i = 0; i < joint_config->count(); ++i)
  {
    if (!tobas::isServoJoint(joint_config->getRole(i)))
      continue;

    const auto jnt_name = joint_config->getJointName(i).toStdString();
    const auto ctrl_name = jnt_name + "_controller";
    manager_params_node[ctrl_name]["type"] = tobas::ctrl_manager::type::kForwardCommandController;
  }

  // Create data
  YAML::Node root_node(YAML::NodeType::Map);
  root_node[robot_.robotName()]["controller_manager"][kROSParamsKey] = manager_params_node;

  // Save data
  if (!saveYamlNode(config_dir / "joint_controller_manager.yaml", root_node))
    return false;

  return true;
}

bool PackageGenerator::generateJointControllerConfigs(const fs::path& config_dir)
{
  const auto joint_config = settings_->joint_config;

  for (int i = 0; i < joint_config->count(); ++i)
  {
    if (!tobas::isServoJoint(joint_config->getRole(i)))
      continue;

    const auto jnt_name = joint_config->getJointName(i).toStdString();
    const auto ctrl_name = jnt_name + "_controller";

    YAML::Node ctrl_params_node(YAML::NodeType::Map);
    ctrl_params_node["joints"].push_back(jnt_name);
    ctrl_params_node["interface_name"] = tobas::jntCmdIfaceEnumToText(joint_config->getCommandInterface(i));

    // Create data
    YAML::Node root_node(YAML::NodeType::Map);
    root_node["/**"][ctrl_name][kROSParamsKey] = ctrl_params_node;  // XXX: 名前空間を指定すると読み込みに失敗する

    // Save data
    if (!saveYamlNode(config_dir / (ctrl_name + ".yaml"), root_node))
      return false;
  }

  return true;
}

bool PackageGenerator::generateDroneConfig(const fs::path& config_dir)
{
  const auto drone = createDrone();

  if (!drone.save(config_dir / "drone.tbsdrn"))
  {
    qt::qErrorBox(settings_, "Failed to save drone configuration.");
    return false;
  }

  return true;
}

bool PackageGenerator::generateRCTeleopConfig(const fs::path& config_dir)
{
  // ComposableNodeにパラメータを渡す際は，<node_name>/ros__parameters以下ではなくルート以下に直接パラメータを書く．
  YAML::Node root_node(YAML::NodeType::Map);
  root_node["stabilize_mode"] = static_cast<int>(settings_->controller->stabilizeModeCommand());
  root_node["acrobat_mode"] = static_cast<int>(settings_->controller->acrobatModeCommand());

  if (!saveYamlNode(config_dir / "rc_teleop.yaml", root_node))
    return false;

  return true;
}

bool PackageGenerator::generateControllerStaticConfig(const fs::path& config_dir)
{
  const auto node = settings_->controller->staticParams();
  TOBAS_CHECK(node.IsMap());

  if (!saveYamlNode(config_dir / "controller_static.yaml", node))
    return false;

  return true;
}

bool PackageGenerator::generateObserverStaticConfig(const fs::path& config_dir)
{
  const auto node = settings_->observer->staticParams();
  TOBAS_CHECK(node.IsMap());

  if (!saveYamlNode(config_dir / "observer_static.yaml", node))
    return false;

  return true;
}

bool PackageGenerator::generateURDF(const fs::path& mesh_dir)
{
  // Export the original URDF
  // コメントやGazeboプラグインなどの不確定要素を排するため，テキストそのままではなく一度URDFオブジェクトを介してエクスポートする．
  const auto doc = urdf::exportURDF(*robot_.urdf());

  // Get robot element
  const auto robot = doc->RootElement();
  if (strcmp(robot->Name(), "robot") != 0)
  {
    qt::qErrorBox(settings_, "Robot description is invalid.");
    return false;
  }

  // Modify robot
  if (!removePropellerJointLimits(robot))
    return false;
  if (!resolveMeshFiles(robot, mesh_dir))
    return false;
  if (!addXMLElements(robot))
    return false;

  // Save modified URDF
  if (doc->SaveFile(common::getModifiedURDFPath(tbsPath()).c_str()) != tinyxml2::XML_SUCCESS)
  {
    qt::qErrorBox(settings_, "Failed to save the modified URDF.");
    return false;
  }

  return true;
}

bool PackageGenerator::createEmptyFile(const fs::path& file_path)
{
  if (!path::createFilePath(file_path))
  {
    qt::qErrorBox(settings_, "Failed to create \"" + QString::fromStdString(file_path) + "\".");
    return false;
  }

  return true;
}

bool PackageGenerator::createEmptyYaml(const fs::path& file_path, bool overwrite)
{
  if (!overwrite && fs::is_regular_file(file_path))
    return true;

  if (!saveYamlNode(file_path, YAML::Node(YAML::NodeType::Map)))
    return false;

  return true;
}

bool PackageGenerator::saveYamlNode(const fs::path& path, const YAML::Node& node)
{
  if (!yaml::save(path, node))
  {
    qt::qErrorBox(settings_, "Failed to save \"" + QString::fromStdString(path) + "\".");
    return false;
  }

  return true;
}

bool PackageGenerator::removePropellerJointLimits(tinyxml2::XMLElement* robot)
{
  std::set<std::string> prop_jnt_names;
  const auto propulsions = settings_->propulsion_system->selected();
  for (int i = 0; i < propulsions->count(); ++i)
  {
    const auto link_name = propulsions->linkName(i).toStdString();
    const auto jnt_name = robot_.tree().getSegment(link_name)->second.segment.joint().name;
    prop_jnt_names.insert(jnt_name);
  }

  for (auto child = robot->FirstChildElement(); child != nullptr; child = child->NextSiblingElement())
  {
    if (strcmp(child->Name(), "joint") == 0)
    {
      const auto jnt_name = child->Attribute("name");
      if (jnt_name == nullptr)
      {
        qt::qErrorBox(settings_, "Joint element does not have attribute: \"name\"");
        return false;
      }
      if (prop_jnt_names.contains(jnt_name))
      {
        for (auto gchild = child->FirstChildElement(); gchild != nullptr; gchild = gchild->NextSiblingElement())
        {
          if (strcmp(gchild->Name(), "limit") == 0)
          {
            child->DeleteChild(gchild);
            break;
          }
        }
      }
    }
  }

  return true;
}

bool PackageGenerator::resolveMeshFiles(tinyxml2::XMLElement* elem, const fs::path& mesh_dir)
{
  if (strcmp(elem->Name(), "mesh") == 0)
  {
    const auto filename = elem->Attribute("filename");
    if (filename == nullptr)
    {
      qt::qErrorBox(settings_, "Mesh element does not have attribute: \"filename\"");
      return false;
    }

    const auto src_path = ros2::resolveURI(filename);
    const auto base_name = src_path.filename();
    const auto dst_path = mesh_dir / base_name;
    if (src_path != dst_path)
    {
      if (fs::exists(dst_path))
      {
        if (!fs::remove(dst_path))
        {
          qt::qErrorBox(settings_, QString::fromStdString(dst_path) + " already exists, but failed to overwrite it.");
          return false;
        }
      }

      // メッシュファイルをTobasパッケージ以下にコピー
      if (!fs::copy_file(src_path, dst_path))
      {
        qt::qErrorBox(
          settings_,
          "Failed to copy " + QString::fromStdString(src_path) + " to " + QString::fromStdString(dst_path) + ".");
        return false;
      }

      // メッシュファイルへのパスを置換
      // package://<pkg_name>の書式だとIgnitionが発見できないため，絶対パスに置換できるようxacroコマンドを埋め込む．
      // cf. https://github.com/moveit/moveit_resources/blob/ros2/panda_description/urdf/panda.urdf.xacro
      const auto config_pkg_name = common::getTBSConfigName(tbsPath());
      const auto new_filename = "file://$(find " + config_pkg_name + ")/meshes/" + base_name.string();
      elem->SetAttribute("filename", new_filename.c_str());
    }
  }

  // 再帰的に子要素もチェック
  for (auto child = elem->FirstChildElement(); child != nullptr; child = child->NextSiblingElement())
    if (!resolveMeshFiles(child, mesh_dir))
      return false;

  return true;
}

bool PackageGenerator::addXMLElements(tinyxml2::XMLElement* robot)
{
  const auto& ns = robot_.robotName();
  const auto& root_name = robot_.tree().getRootName();
  const auto config_pkg_name = common::getTBSConfigName(tbsPath());

  const auto& batt = settings_->battery;
  const auto& propulsions = settings_->propulsion_system->selected();
  const auto& fixed_wing = settings_->fixed_wing;
  const auto& imu = settings_->imu;
  const auto& mag = settings_->magnetometer;
  const auto& baro = settings_->barometer;
  const auto& gps = settings_->gps;
  const auto& sim = settings_->simulation;

  const auto drone = createDrone();

  // Get rotor channels
  vector<size_t> rotor_channels;
  for (const auto& [channel, _] : drone.rotors)
    rotor_channels.push_back(channel);

  // XML namespace
  robot->SetAttribute("xmlns:xacro", "http://ros.org/wiki/xacro");

  // Battery plugin
  constexpr double kBatterySamplingRate = 100.;  // TODO: サンプリングレートをGUIで設定
  addBatteryPlugin(
    robot, ns, kBatterySamplingRate, batt->maxVoltage(), batt->sagVoltage(), batt->maxCurrent(), batt->capacity(),
    batt->internalRegistance(), rotor_channels);

  // IMU plugin
  addIMUPlugin(
    robot, ns, root_name, imu->updateRate(), imu->offset(), imu->gyroNoiseDensity(), imu->gyroRandomWalk(),
    imu->gyroBiasCorrTime(), imu->gyroTurnOnBiasSigma(), imu->accNoiseDensity(), imu->accRandomWalk(),
    imu->accBiasCorrTime(), imu->accTurnOnBiasSigma(), rotor_channels);

  // Magnetometer plugin
  addMagnetometerPlugin(
    robot, ns, root_name, mag->updateRate(), mag->offset(), sim->latitudeZero(), sim->longitudeZero(),
    sim->altitudeZero(), mag->noiseStddev(), mag->hardBiasRange());

  // Barometer plugin
  addBarometerPlugin(
    robot, ns, root_name, baro->updateRate(), baro->offset(), sim->altitudeZero(), baro->pressureVariance());

  // GPS plugin
  addGPSPlugin(
    robot, ns, root_name, gps->updateRate(), gps->offset(), gps->delay(), gps->positionCorrectionTime(),
    gps->horizontalPositionAccuracy(), gps->verticalPositionAccuracy(), gps->horizontalVelocityStddev(),
    gps->verticalVelocityStddev(), sim->latitudeZero(), sim->longitudeZero(), sim->altitudeZero());

  // Rotor plugins
  for (int i = 0; i < propulsions->count(); ++i)
  {
    const auto link_name = propulsions->linkName(i).toStdString();

    const auto propulsion = propulsions->widget(i);
    const auto general = propulsion->general();
    const auto esc = propulsion->esc();
    const auto motor = propulsion->motor();
    const auto propeller = propulsion->propeller();
    const auto aero = propulsion->aerodynamics();

    addRotorPlugin(
      robot, ns, link_name, general->channel(), motor->kv(), motor->internalResistance(), propeller->numBlade(),
      aero->motorConst(), aero->momentConst(), aero->rotorDragCoef(), motor->direction(), esc->maxCurrent(),
      sim->maxModelErrorRate());
  }

  // Fixed wing plugin
  if (fixed_wing->hasFixedWing())
    addFixedWingPlugin(robot, ns, root_name, sim->altitudeZero(), drone.fixed_wing);

  // Wind plugin
  addGazeboWindPlugin(robot, ns, root_name);

  // Ground truth state plugin
  addGazeboGroundTruthStatePlugin(robot, ns, root_name);

  // Gazebo ROS2 control system
  addGazeboROS2SimSystem(robot, drone.joints);

  // Gazebo ROS2 control plugin
  // XXX: This must be defined after GazeboSimSystem
  if (hasServoJoint())
    addGazeboSimROS2ControlPlugin(robot, ns, config_pkg_name, "config/joint_controller_manager.yaml");

  // Base static joint for debug
  addBaseStaticJoint(robot, robot_.tree().getRootName());

  return true;
}
}  // namespace setup_assistant
}  // namespace gui
