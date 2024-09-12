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
#include <tobas_tools/package.hpp>
#include <tobas_tools/command.hpp>
#include <tobas_qt_tools/message.hpp>

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
  cfg_env_ = make_shared<TemplateGenerator>(templates_path / "config_package");
  user_env_ = make_shared<TemplateGenerator>(templates_path / "user_package");
}

bool PackageGenerator::generatePackage()
{
  // Tobasパッケージを作成
  if (!path::createDirectories(tbsPath()))
  {
    qt::qErrorBox(settings_, "Failed to create Tobas package path.");
    return false;
  }

  // テンプレート用アイテムを作成
  const auto tpl_data = createTemplateData();

  // メタパッケージを作成
  if (!generateMetaPackage(tpl_data))
    return false;

  // 設定パッケージを作成
  if (!generateConfigPackage(tpl_data))
    return false;

  // ユーザパッケージを作成
  if (!generateUserPackage(tpl_data))
    return false;

  return true;
}

string PackageGenerator::tbsPath() const
{
  return settings_->ros_package->tbsPath().toStdString();
}

inja::json PackageGenerator::createTemplateData()
{
  inja::json tpl_data;

  tpl_data["drone_name"] = robot_.robotName();

  // Controller
  tpl_data["controller_pkg"] = settings_->controller->controllerPackage();
  tpl_data["actions_pkg"] = settings_->controller->actionsPackage();

  // Observer
  tpl_data["observer_pkg"] = settings_->observer->observerPackage();

  // Hardware
  tpl_data["hardware_pkg"] = settings_->hardware->hardwarePackage();

  // Author Info
  tpl_data["author_name"] = settings_->author_info->authorName().toStdString();
  tpl_data["author_email"] = settings_->author_info->authorEmail().toStdString();

  // Ros Package
  tpl_data["meta_pkg_name"] = tobas::getTBSMetaName(tbsPath());
  tpl_data["config_pkg_name"] = tobas::getTBSConfigName(tbsPath());
  tpl_data["user_pkg_name"] = tobas::getTBSUserName(tbsPath());

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
  const auto servos = settings_->servo_joints->selected();
  for (int i = 0; i < servos->count(); ++i)
  {
    tobas::JointConfig joint;
    joint.name = servos->jointName(i).toStdString();
    joint.home_pos = servos->homePosition(i);
    joint.min_pos = servos->minPosition(i);
    joint.max_pos = servos->maxPosition(i);
    joint.interface = servos->interface(i);

    drone.joints[joint.name] = joint;
  }

  // Rotors
  const auto props = settings_->propulsion_system->selected();
  const auto num_rotors = props->count();
  drone.rotors.resize(num_rotors);
  for (int i = 0; i < num_rotors; ++i)
  {
    const auto link_name = props->linkName(i).toStdString();
    const auto prop_config = props->widget(i);
    drone.rotors.at(i).channel = i;
    drone.rotors.at(i).link_name = link_name;
    drone.rotors.at(i).direction = prop_config->motor()->direction();
    drone.rotors.at(i).axis = robot_.rotorAxisType(link_name);
    drone.rotors.at(i).esc_mode = prop_config->esc()->signalMode();
    drone.rotors.at(i).num_poles = prop_config->motor()->numPoles();
    drone.rotors.at(i).max_rot_speed = prop_config->speedLimit()->maxRotSpeed();
    drone.rotors.at(i).motor_constant = prop_config->aerodynamics()->motorConst();
    drone.rotors.at(i).moment_constant = prop_config->aerodynamics()->momentConst();
    drone.rotors.at(i).drag_constant = prop_config->aerodynamics()->rotorDragCoef();
    drone.rotors.at(i).rot_speed_coefs = prop_config->electrodynamics()->rotSpeedCoefs();
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
    const auto num_cs = css->count();
    drone.fixed_wing.control_surfaces.resize(num_cs);
    for (int i = 0; i < num_cs; ++i)
    {
      drone.fixed_wing.control_surfaces.at(i).channel = i;
      drone.fixed_wing.control_surfaces.at(i).joint_name = css->jointName(i).toStdString();
      drone.fixed_wing.control_surfaces.at(i).angle_limit.lower = css->minAngle(i);
      drone.fixed_wing.control_surfaces.at(i).angle_limit.upper = css->maxAngle(i);
      drone.fixed_wing.control_surfaces.at(i).max_angle_rate = css->maxAngleRate(i);
      drone.fixed_wing.control_surfaces.at(i).c_lift_delta = css->liftCoef(i);
      drone.fixed_wing.control_surfaces.at(i).c_drag_abs_delta = css->dragCoef(i);  // FIXME: 正負の確認が必要？
      drone.fixed_wing.control_surfaces.at(i).c_side_delta = css->sideCoef(i);
      drone.fixed_wing.control_surfaces.at(i).c_roll_delta = css->rollCoef(i);
      drone.fixed_wing.control_surfaces.at(i).c_pitch_delta = css->pitchCoef(i);
      drone.fixed_wing.control_surfaces.at(i).c_yaw_delta = css->yawCoef(i);
    }
  }

  return drone;
}

bool PackageGenerator::generateMetaPackage(const inja::json& tpl_data)
{
  const auto meta_pkg_path = tobas::getTBSMetaPath(tbsPath());
  fs::create_directory(meta_pkg_path);

  meta_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", meta_pkg_path);
  meta_env_->generate(tpl_data, "package.xml.tplxml", meta_pkg_path);

  if (!createEmptyFile(meta_pkg_path / kDoNotEditThisPackage))
    return false;

  return true;
}

bool PackageGenerator::generateConfigPackage(const inja::json& tpl_data)
{
  const auto config_pkg_path = tobas::getTBSConfigPath(tbsPath());
  fs::create_directory(config_pkg_path);

  // ディレクトリを作成
  const auto backup_dir = config_pkg_path / "backup";
  const auto config_dir = config_pkg_path / "config";
  const auto launch_dir = config_pkg_path / "launch";
  const auto urdf_dir = config_pkg_path / "urdf";
  const auto mesh_dir = config_pkg_path / "meshes";
  fs::create_directory(backup_dir);
  fs::create_directory(config_dir);
  fs::create_directory(launch_dir);
  fs::create_directory(urdf_dir);
  fs::create_directory(mesh_dir);

  // バックアップ用ファイル
  const auto backup_data = settings_->dump();
  if (!saveYamlNode(tobas::getSettingsPath(tbsPath()), backup_data))
    return false;

  // テンプレートから生成
  cfg_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", config_pkg_path);
  cfg_env_->generate(tpl_data, "package.xml.tplxml", config_pkg_path);
  cfg_env_->generate(tpl_data, "gazebo.launch.xml.tplxml", launch_dir);
  cfg_env_->generate(tpl_data, "real.launch.xml.tplxml", launch_dir);
  cfg_env_->generate(tpl_data, "core_component.launch.py.tplpy", launch_dir);
  cfg_env_->generate(tpl_data, "core_standalone.launch.py.tplpy", launch_dir);
  cfg_env_->generate(tpl_data, "hil.launch.xml.tplxml", launch_dir);
  cfg_env_->generate(tpl_data, "robot_state_publisher.launch.py.tplpy", launch_dir);

  // Keyboard Teleop (コントローラの対応コマンドによって場合分け)
  // TODO: コントローラごとに1つずつ
  if (
    settings_->controller->isCommandCompatible(tobas::rc_command_t::POSITION_YAW)
    || settings_->controller->isCommandCompatible(tobas::rc_command_t::POS_VEL_ACC_YAW))
  {
    cfg_env_->generate(tpl_data, "keyboard_teleop/position_yaw/keyboard_teleop.launch.py.tplpy", launch_dir);
  }
  else if (settings_->controller->isCommandCompatible(tobas::rc_command_t::SPEED_ROLL_DPITCH))
  {
    cfg_env_->generate(tpl_data, "keyboard_teleop/speed_roll_dpitch/keyboard_teleop.launch.py.tplpy", launch_dir);
  }

  // GUI Teleop (コントローラの対応コマンドによって場合分け)
  // TODO: コントローラごとに1つずつ
  if (
    settings_->controller->isCommandCompatible(tobas::rc_command_t::POSITION_YAW)
    || settings_->controller->isCommandCompatible(tobas::rc_command_t::POS_VEL_ACC_YAW)
    || settings_->controller->isCommandCompatible(tobas::rc_command_t::POSE_TWIST_ACCEL))
  {
    cfg_env_->generate(tpl_data, "gui_teleop/position_yaw/gui_teleop.launch.py.tplpy", launch_dir);
  }

  // Dynamic parameters
  if (!createEmptyYaml(config_dir / "controller_dynamic.yaml", false))
    return false;
  if (!createEmptyYaml(config_dir / "observer_dynamic.yaml", false))
    return false;

  // その他
  if (!createEmptyFile(config_pkg_path / kDoNotEditThisPackage))
    return false;
  if (!generateControllerManagerLaunch(launch_dir))
    return false;
  if (!generateGazeboJointCommandHandlerConfig(config_dir))
    return false;
  if (!generateJointControlConfig(config_dir))
    return false;
  if (!generateDroneConfig(config_dir))
    return false;
  if (!generateRCTeleopConfig(config_dir))
    return false;
  if (!generateControllerStaticConfig(config_dir))
    return false;
  if (!generateObserverStaticConfig(config_dir))
    return false;
  if (!generateURDFs(mesh_dir))
    return false;

  return true;
}

bool PackageGenerator::generateUserPackage(const inja::json& tpl_data)
{
  const auto user_pkg_path = tobas::getTBSUserPath(tbsPath());
  fs::create_directory(user_pkg_path);

  // ディレクトリを作成
  const auto launch_dir = user_pkg_path / "launch";
  const auto nodes_dir = user_pkg_path / "nodes";
  fs::create_directory(launch_dir);
  fs::create_directory(nodes_dir);

  // テンプレートから作成 (存在する場合は上書きしない)
  user_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", user_pkg_path, false);
  user_env_->generate(tpl_data, "package.xml.tplxml", user_pkg_path, false);
  user_env_->generate(tpl_data, "common.launch.py.tplpy", launch_dir, false);
  user_env_->generate(tpl_data, "gazebo.launch.py.tplpy", launch_dir, false);
  user_env_->generate(tpl_data, "real.launch.py.tplpy", launch_dir, false);
  user_env_->generate(tpl_data, "tobas_bridge_node.cpp.tplcpp", nodes_dir, false);

  // その他
  if (!createEmptyFile(user_pkg_path / kYouCanEditThisPackage))
    return false;

  return true;
}

bool PackageGenerator::generateControllerManagerLaunch(const fs::path& launch_dir)
{
  const auto servos = settings_->servo_joints->selected();

  // XMLを作成
  const auto doc = new tinyxml2::XMLDocument();
  const auto launch = doc->NewElement("launch");
  doc->InsertFirstChild(launch);

  // サーボジョイントが少なくとも1つ登録されている場合に限りcontroller_managerを立ち上げる
  if (servos->count() > 0)
  {
    const auto config_pkg_name = tobas::getTBSConfigName(tbsPath());
    const auto param_file = "$(find-pkg-share " + config_pkg_name + ")/config/joint_control.yaml";

    // Joint state broadcaster
    const auto jsb_node = addNode(launch, "controller_manager", "spawner", "", "", "joint_state_broadcaster");
    addNodeParam(jsb_node, "use_sim_time", "true");

    // コントローラごとにノードを立ち上げる
    for (int i = 0; i < servos->count(); ++i)
    {
      const auto controller_name = servos->jointName(i).toStdString() + "_controller";
      const auto args = controller_name + "  --param-file " + param_file;
      const auto ctrl_node = addNode(launch, "controller_manager", "spawner", "", "", args);
      addNodeParam(ctrl_node, "use_sim_time", "true");
    }
  }

  // XMLを保存
  if (doc->SaveFile((launch_dir / "controller_manager.launch.xml").c_str()) != tinyxml2::XML_SUCCESS)
  {
    qt::qErrorBox(settings_, "Failed to save the controller manager configurations.");
    return false;
  }

  return true;
}

bool PackageGenerator::generateGazeboJointCommandHandlerConfig(const std::filesystem::path& config_dir)
{
  const auto servos = settings_->servo_joints->selected();

  // 空の配列はyamlを読み込んだ時点で"No parameter value set"エラーが出るため，長さが0ならパラメータ自体を設定しない．
  YAML::Node params_node(YAML::NodeType::Map);
  for (int i = 0; i < servos->count(); ++i)
  {
    params_node["joint_names"].push_back(servos->jointName(i));
    params_node["interfaces"].push_back(servos->interface(i));
  }

  YAML::Node root_node(YAML::NodeType::Map);
  root_node[robot_.robotName()]["gazebo_joint_command_handler"][kROSParamsKey] = params_node;

  if (!saveYamlNode(config_dir / "gazebo_joint_command_handler.yaml", root_node))
    return false;

  return true;
}

bool PackageGenerator::generateJointControlConfig(const fs::path& config_dir)
{
  // cf. https://github.com/ros-controls/gz_ros2_control/tree/rolling/gz_ros2_control_demos/config

  // Create data
  YAML::Node root_node(YAML::NodeType::Map);

  // Controller manager
  YAML::Node manager_params_node(YAML::NodeType::Map);
  manager_params_node["update_rate"] = 1000;  // TODO: GUIで設定できるように
  manager_params_node["joint_state_broadcaster"]["type"] = tobas::controller_manager::type::kJointStateBroadcaster;
  root_node[robot_.robotName()]["controller_manager"][kROSParamsKey] = manager_params_node;

  // Each joint controllers
  const auto servos = settings_->servo_joints->selected();
  for (int i = 0; i < servos->count(); ++i)
  {
    const auto jnt_name = servos->jointName(i);
    const auto ctrl_name = jnt_name + "_controller";

    YAML::Node controller_node(YAML::NodeType::Map);
    controller_node["type"] = tobas::controller_manager::type::kForwardCommandController;
    controller_node["joints"].push_back(jnt_name);
    controller_node["interface_name"] = tobas::jointIFEnumToText(servos->interface(i));

    root_node[robot_.robotName()][ctrl_name][kROSParamsKey] = controller_node;
  }

  // Save data
  if (!saveYamlNode(config_dir / "joint_control.yaml", root_node))
    return false;

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

bool PackageGenerator::generateURDFs(const fs::path& mesh_dir)
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

  // Save original URDF
  if (doc->SaveFile(tobas::getOriginalURDFPath(tbsPath()).c_str()) != tinyxml2::XML_SUCCESS)
  {
    qt::qErrorBox(settings_, "Failed to save the original URDF.");
    return false;
  }

  // Modify robot
  if (!resolveMeshFiles(robot, mesh_dir))
    return false;
  if (!addXMLElements(robot))
    return false;

  // Save modified URDF
  if (doc->SaveFile(tobas::getModifiedURDFPath(tbsPath()).c_str()) != tinyxml2::XML_SUCCESS)
  {
    qt::qErrorBox(settings_, "Failed to save the original URDF.");
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
      const auto config_pkg_name = tobas::getTBSConfigName(tbsPath());
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

  const auto& batt = settings_->battery;
  const auto& props = settings_->propulsion_system->selected();
  const auto& fixed_wing = settings_->fixed_wing;
  const auto& joints = settings_->servo_joints->selected();
  const auto& imu = settings_->imu;
  const auto& mag = settings_->magnetometer;
  const auto& baro = settings_->barometer;
  const auto& gps = settings_->gps;
  const auto& sim = settings_->simulation;

  const auto drone = createDrone();

  // XML namespace
  robot->SetAttribute("xmlns:xacro", "http://ros.org/wiki/xacro");

  // Battery plugin
  constexpr double kBatterySamplingRate = 100.;  // TODO: サンプリングレートをGUIで設定
  addBatteryPlugin(
    robot, ns, kBatterySamplingRate, batt->maxVoltage(), batt->sagVoltage(), batt->maxCurrent(), batt->capacity(),
    batt->internalRegistance(), props->count());

  // IMU plugin
  addIMUPlugin(
    robot, ns, root_name, imu->updateRate(), imu->offset(), imu->gyroNoiseDensity(), imu->gyroRandomWalk(),
    imu->gyroBiasCorrTime(), imu->gyroTurnOnBiasSigma(), imu->gyroLPFCutoffFreq(), imu->accNoiseDensity(),
    imu->accRandomWalk(), imu->accBiasCorrTime(), imu->accTurnOnBiasSigma(), imu->accLPFCutoffFreq());

  // Magnetometer plugin
  addMagnetometerPlugin(
    robot, ns, root_name, mag->updateRate(), mag->offset(), sim->latitudeZero(), sim->longitudeZero(),
    sim->altitudeZero(), mag->gaussNoise(), mag->uniformNoise());

  // Barometer plugin
  addBarometerPlugin(
    robot, ns, root_name, baro->updateRate(), baro->offset(), sim->altitudeZero(), baro->pressureVariance());

  // GPS plugin
  addGPSPlugin(
    robot, ns, root_name, gps->updateRate(), gps->offset(), gps->delay(), gps->positionCorrectionTime(),
    gps->horizontalPositionAccuracy(), gps->verticalPositionAccuracy(), gps->horizontalVelocityStddev(),
    gps->verticalVelocityStddev(), sim->latitudeZero(), sim->longitudeZero(), sim->altitudeZero());

  // Rotor plugins
  for (int i = 0; i < props->count(); ++i)
  {
    const auto link_name = props->linkName(i).toStdString();
    const auto jnt_name = robot_.tree().getSegment(link_name)->second.segment.joint().name;

    const auto prop = props->widget(i);
    const auto motor = prop->motor();
    const auto esc = prop->esc();

    addRotorPlugin(
      robot, ns, jnt_name, drone.rotors.at(i), motor->timeConstUp(), motor->timeConstDown(), esc->maxCurrent(),
      sim->maxModelErrorRate());
  }

  // Fixed wing plugin
  if (fixed_wing->hasFixedWing())
    addFixedWingPlugin(robot, ns, root_name, sim->altitudeZero(), drone.fixed_wing);

  // Wind plugin
  addGazeboWindPlugin(robot, ns, root_name);

  // Ground truth state plugin
  addGazeboGroundTruthStatePlugin(robot, ns, root_name);

  // Rotor speeds publisher plugin
  vector<string> rotor_jnt_names;
  for (int i = 0; i < props->count(); ++i)
  {
    const auto link_name = props->linkName(i).toStdString();
    const auto jnt_name = robot_.tree().getSegment(link_name)->second.segment.joint().name;
    rotor_jnt_names.push_back(jnt_name);
  }
  addRotorSpeedsPublisherPlugin(robot, ns, rotor_jnt_names);

  // Gazebo ROS2 control plugin
  // FIXME: ジョイントが1つも設定されてないとフリーズする？
  if (joints->count() > 0)
    addGazeboSimROS2ControlPlugin(robot, ns, tobas::getTBSConfigName(tbsPath()), "config/joint_control.yaml");

  // Gazebo ROS2 control system
  addGazeboROS2SimSystem(robot, drone.joints);

  // Base static joint for debug
  addBaseStaticJoint(robot, robot_.tree().getRootName());

  return true;
}
}  // namespace setup_assistant
}  // namespace gui
