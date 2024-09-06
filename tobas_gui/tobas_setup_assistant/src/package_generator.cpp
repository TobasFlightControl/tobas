#include <rclcpp/wait_for_message.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>

#include <QDebug>
#include <QMessageBox>

#include <tobas_std_tools/check.hpp>
#include <tobas_path_tools/core.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_ros2_tools/path.hpp>
#include <tobas_tools/package.hpp>
#include <tobas_tools/command.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>

#include "tobas_setup_assistant/package_generator.hpp"
#include "tobas_setup_assistant/xml_nodes.hpp"

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

void PackageGenerator::generate()
{
  qt::ProgressDialog progress("Generate Package", 3, settings_);
  progress.setCancelButton(nullptr);
  progress.show();

  // Verify user settings
  progress.setLabelText("Verifying the validity of the user settings.");
  if (settings_->isValid())
  {
    progress.close();
    return;
  }
  progress.progressStep();

  // Generate Tobas package
  progress.setLabelText("Generating Tobas package.");
  if (!generatePackage())
  {
    progress.close();
    return;
  }
  progress.progressStep();

  // Build Tobas package
  progress.setLabelText("Building Tobas package.");
  if (!tobas::buildTobasPackage(tbsPath()))
  {
    progress.close();
    qt::qErrorBox(settings_, "Failed to build Tobas package.");
    return;
  }
  progress.progressStep();

  progress.close();
  qt::qInfoBox(settings_, "Tobas configuration package is generated and built successfully.");
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

  // Joint Controllers
  string joint_controllers = "joint_state_controller";
  for (int i = 0; i < settings_->custom_joints->count(); ++i)
  {
    const auto jnt_name = settings_->custom_joints->getJointName(i);
    joint_controllers += " " + jnt_name.toStdString() + "_controller";
  }
  tpl_data["joint_controllers"] = joint_controllers;

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
  for (int i = 0; i < settings_->custom_joints->count(); ++i)
  {
    tobas::JointConfig joint;
    joint.name = settings_->custom_joints->getJointName(i).toStdString();
    joint.home_pos = settings_->custom_joints->getHomePosition(i);
    joint.min_pos = settings_->custom_joints->getMinPosition(i);
    joint.max_pos = settings_->custom_joints->getMaxPosition(i);
    joint.control_type = settings_->custom_joints->getCommandType(i);

    drone.joints[joint.name] = joint;
  }

  // Rotors
  const auto selected_props = settings_->propulsion_system->selected();
  const auto num_rotors = selected_props->count();
  drone.rotors.resize(num_rotors);
  for (int i = 0; i < num_rotors; ++i)
  {
    const auto link_name = selected_props->linkName(i).toStdString();
    const auto prop_config = selected_props->widget(i);
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
    const auto num_cs = css->rowCount();
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

bool PackageGenerator::generatePackage()
{
  // Tobasパッケージを作成
  fs::create_directories(tbsPath());

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
  const auto mesh_dir = config_pkg_path / "mesh";
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
  cfg_env_->generate(tpl_data, "component_manager_high.launch.xml.tplxml", launch_dir);
  cfg_env_->generate(tpl_data, "component_manager_medium.launch.xml.tplxml", launch_dir);
  cfg_env_->generate(tpl_data, "component_manager_low.launch.xml.tplxml", launch_dir);
  cfg_env_->generate(tpl_data, "gazebo.launch.xml.tplxml", launch_dir);
  cfg_env_->generate(tpl_data, "real.launch.xml.tplxml", launch_dir);
  cfg_env_->generate(tpl_data, "bringup.launch.py.tplpy", launch_dir);
  cfg_env_->generate(tpl_data, "hil.launch.xml.tplxml", launch_dir);
  cfg_env_->generate(tpl_data, "robot_state_publisher.launch.xml.tplxml", launch_dir);

  // Keyboard Teleop (コントローラの対応コマンドによって場合分け)
  // TODO: コントローラごとに1つずつ
  if (
    settings_->controller->isCommandCompatible(tobas::rc_command_t::POSITION_YAW)
    || settings_->controller->isCommandCompatible(tobas::rc_command_t::POS_VEL_ACC_YAW))
  {
    cfg_env_->generate(tpl_data, "keyboard_teleop/position_yaw/keyboard_teleop.launch.tplxml", launch_dir);
  }
  else if (settings_->controller->isCommandCompatible(tobas::rc_command_t::SPEED_ROLL_DPITCH))
  {
    cfg_env_->generate(tpl_data, "keyboard_teleop/speed_roll_dpitch/keyboard_teleop.launch.tplxml", launch_dir);
  }

  // GUI Teleop (コントローラの対応コマンドによって場合分け)
  // TODO: コントローラごとに1つずつ
  if (
    settings_->controller->isCommandCompatible(tobas::rc_command_t::POSITION_YAW)
    || settings_->controller->isCommandCompatible(tobas::rc_command_t::POS_VEL_ACC_YAW)
    || settings_->controller->isCommandCompatible(tobas::rc_command_t::POSE_TWIST_ACCEL))
  {
    cfg_env_->generate(tpl_data, "gui_teleop/position_yaw/gui_teleop.launch.tplxml", launch_dir);
  }

  // その他
  if (!createEmptyFile(config_pkg_path / kDoNotEditThisPackage))
    return false;
  if (!createEmptyFile(config_dir / "dynamic_params.yaml"))
    return false;
  if (!generateDroneConfig(config_dir))
    return false;
  if (!generateJointControlConfig(config_dir))
    return false;
  if (!generateRCTeleopConfig(config_dir))
    return false;
  if (!generateControllerConfig(config_dir))
    return false;
  if (!generateObserverConfig(config_dir))
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
  cfg_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", user_pkg_path, false);
  cfg_env_->generate(tpl_data, "package.xml.tplxml", user_pkg_path, false);
  cfg_env_->generate(tpl_data, "common.launch.py.tplpy", launch_dir, false);
  cfg_env_->generate(tpl_data, "gazebo.launch.py.tplpy", launch_dir, false);
  cfg_env_->generate(tpl_data, "real.launch.py.tplpy", launch_dir, false);
  cfg_env_->generate(tpl_data, "tobas_bridge_node.cpp.tplcpp", nodes_dir, false);

  // その他
  if (!createEmptyFile(user_pkg_path / kYouCanEditThisPackage))
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

bool PackageGenerator::generateJointControlConfig(const fs::path& config_dir)
{
  // Create data
  YAML::Node rosparam_node(YAML::NodeType::Map);

  // joint_state_broadcaster
  YAML::Node jsb_node(YAML::NodeType::Map);
  jsb_node["publish_rate"] = 1000;
  jsb_node["type"] = "joint_state_broadcaster/JointStateBroadcaster";

  // Each joint controllers
  for (int i = 0; i < settings_->custom_joints->count(); ++i)
  {
    const auto jnt_name = settings_->custom_joints->getJointName(i).toStdString();
    const auto controller_name = jnt_name + "_controller";

    YAML::Node controller_node(YAML::NodeType::Map);
    controller_node["type"] = settings_->custom_joints->getControllerType(i).toStdString();
    controller_node["joints"].push_back(jnt_name);

    rosparam_node[controller_name] = controller_node;
  }

  YAML::Node root_node(YAML::NodeType::Map);
  root_node["controller_manager"][kROSParamsKey] = rosparam_node;

  // Save data
  if (!saveYamlNode(config_dir / "joint_control.yaml", root_node))
    return false;

  return true;
}

bool PackageGenerator::generateRCTeleopConfig(const fs::path& config_dir)
{
  YAML::Node rosparam_node(YAML::NodeType::Map);
  rosparam_node["stabilize_mode"] = static_cast<int>(settings_->controller->stabilizeModeCommand());
  rosparam_node["acrobat_mode"] = static_cast<int>(settings_->controller->acrobatModeCommand());

  YAML::Node root_node(YAML::NodeType::Map);
  root_node["rc_teleop"][kROSParamsKey] = rosparam_node;

  if (!saveYamlNode(config_dir / "rc_teleop.yaml", root_node))
    return false;

  return true;
}

bool PackageGenerator::generateControllerConfig(const fs::path& config_dir)
{
  YAML::Node root_node(YAML::NodeType::Map);
  root_node[tobas::kControllerNode][kROSParamsKey] = settings_->controller->staticParams();

  if (!saveYamlNode(config_dir / "controller.yaml", root_node))
    return false;

  return true;
}

bool PackageGenerator::generateObserverConfig(const fs::path& config_dir)
{
  YAML::Node root_node(YAML::NodeType::Map);
  root_node[tobas::kObserverNode][kROSParamsKey] = settings_->observer->staticParams();

  if (!saveYamlNode(config_dir / "observer.yaml", root_node))
    return false;

  return true;
}

bool PackageGenerator::generateURDFs(const fs::path& mesh_dir)
{
  // Get robot description
  std_msgs::msg::String robot_description;
  if (!rclcpp::wait_for_message(robot_description, node_, tobas::kRobotDescriptionTopic, 1s))
  {
    qt::qErrorBox(settings_, "Failed to get robot description.");
    return false;
  }

  // Parse robot description
  const auto doc = new tinyxml2::XMLDocument();
  if (!doc->Parse(robot_description.data.c_str()) != tinyxml2::XML_SUCCESS)
  {
    qt::qErrorBox(settings_, "Failed to parse robot description.");
    return false;
  }

  // Get robot element
  const auto robot = doc->RootElement();
  if (!robot->Name())
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
  if (!screenXMLElements(robot))
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
  const auto config_pkg_name = tobas::getTBSConfigName(tbsPath());

  for (auto child = elem->FirstChildElement(); child != nullptr; child = child->NextSiblingElement())
  {
    if (strcmp(elem->Name(), "mesh") == 0)
    {
      const auto filename = elem->Attribute("filename");
      if (filename == nullptr)
      {
        qt::qErrorBox(settings_, "Mesh element does not have attribute: \"filename\"");
        return false;
      }
      const auto src_path = ros2::resolveURI(elem->Attribute("filename"));
      const auto base_name = src_path.filename();
      const auto dst_path = mesh_dir / base_name;
      if (src_path != dst_path)
      {
        if (!fs::copy_file(src_path, dst_path))
        {
          qt::qErrorBox(
            settings_,
            "Failed to copy " + QString::fromStdString(src_path) + " to " + QString::fromStdString(dst_path) + ".");
          return false;
        }
      }
    }

    // 再帰的に子要素もチェック
    if (!resolveMeshFiles(child, mesh_dir))
      return false;
  }

  return true;
}

bool PackageGenerator::screenXMLElements(tinyxml2::XMLElement* robot)
{
  for (auto gazebo = robot->FirstChildElement(); gazebo != nullptr; gazebo = gazebo->NextSiblingElement())
  {
    if (strcmp(gazebo->Name(), "gazebo") == 0)
      continue;

    for (auto plugin = gazebo->FirstChildElement(); plugin != nullptr; plugin = plugin->NextSiblingElement())
    {
      if (strcmp(plugin->Name(), "plugin") == 0)
        continue;

      if (isDeletableGazeboPlugin(plugin) || askRemoveOrKeepGazeboChild(plugin))
        gazebo->DeleteChild(plugin);
    }
  }

  return true;
}

bool PackageGenerator::isDeletableGazeboPlugin(tinyxml2::XMLElement* plugin)
{
  const auto filename = plugin->Attribute("filename");

  // ファイル名が指定されていなければ削除
  if (filename == nullptr)
    return true;

  const string filename_str(filename);

  // Tobasのプラグインは削除
  if (filename_str.starts_with("tobas") || filename_str.starts_with("libtobas"))
    return true;

  // Gazebo ROS2 Controlは削除
  if (filename_str.ends_with("gazebo_ros2_control.so"))
    return true;

  return false;
}

bool PackageGenerator::askRemoveOrKeepGazeboChild(tinyxml2::XMLElement* child)
{
  // メッセージボックスを作成
  const auto msg_box = new QMessageBox(settings_);  // 親を設定しておけば親の開放時に開放される

  // テキストの設定
  const QString tag(child->Name());
  auto text = "Gazebo " + tag + " is detected.\n\n";
  for (auto attr = child->FirstAttribute(); attr != nullptr; attr = attr->Next())
  {
    const QString attr_name(attr->Name());
    const QString attr_value(attr->Value());
    text += "    " + attr_name + ": " + attr_value + "\n";
  }
  text += "\nThis may interfere with components automatically added by Tobas.";
  msg_box->setText(text);
  msg_box->setInformativeText("Do you remove this " + tag + " or keep it?");

  // ボタンの設定
  const auto remove_button = msg_box->addButton("Remove", QMessageBox::ActionRole);
  msg_box->addButton("Keep", QMessageBox::ActionRole);
  msg_box->setDefaultButton(remove_button);

  // ユーザの返事を取得
  msg_box->exec();

  // Removeが選択されたらTrue
  return msg_box->clickedButton() == remove_button;
}

bool PackageGenerator::addXMLElements(tinyxml2::XMLElement* robot)
{
  const auto& ns = robot_.robotName();
  const auto& root_name = robot_.tree().getRootName();

  const auto& batt = settings_->battery;
  const auto& props = settings_->propulsion_system->selected();
  const auto& fixed_wing = settings_->fixed_wing;
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

  // Gazebo ROS control plugin
  addGazeboROSControlPlugin(robot, ns, tobas::getTBSConfigName(tbsPath()), "config/joint_control.yaml");

  // Base static joint for debug
  addBaseStaticJoint(robot, robot_.tree().getRootName());

  return true;
}
}  // namespace setup_assistant
}  // namespace gui
