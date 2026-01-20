#include "tobas_setup_assistant/project_generator.hpp"

#include <tobas_gui_common/command.hpp>
#include <tobas_gui_common/network_config.hpp>
#include <tobas_gui_common/project_paths.hpp>
#include <tobas_gui_common/ssh_config.hpp>
#include <tobas_gui_common/version.hpp>
#include <tobas_math/definitions.hpp>
#include <tobas_path_tools/core.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_string_tools/core.hpp>
#include <tobas_uadf/exporter.hpp>
#include <tobas_urdf/exporter.hpp>
#include <tobas_urdf/util.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_yaml_tools/core.hpp>

#include "tobas_setup_assistant/util.hpp"
#include "tobas_setup_assistant/xml_elements/xml_elements.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sa
{
ProjectGenerator::ProjectGenerator(
  rclcpp::Node::SharedPtr node,
  const uadf::Model& uadf,
  const kdl::Tree& tree,
  const SettingsWidget* settings,
  QWidget* parent)
  : node_(node), uadf_(uadf), tree_(tree), settings_(settings), parent_(parent)
{
  const auto templates_path = getPkgShareDir() / "templates";
  meta_env_ = std::make_shared<TemplateGenerator>(templates_path / "meta_package");
  config_env_ = std::make_shared<TemplateGenerator>(templates_path / "config_package");
  user_msg_env_ = std::make_shared<TemplateGenerator>(templates_path / "user_msg_package");
  user_cpp_env_ = std::make_shared<TemplateGenerator>(templates_path / "user_cpp_package");
  user_py_env_ = std::make_shared<TemplateGenerator>(templates_path / "user_py_package");
}

bool ProjectGenerator::generateProject(const fs::path& proj_path)
{
  // プロジェクトパスを設定
  proj_paths_.setProjPath(proj_path);

  // Tobasパッケージを作成
  const auto create_proj_path_res = path::createDirectories(proj_path);
  if (!create_proj_path_res) {
    qt::qErrorBox(
      parent_, "Failed to create Tobas project path:\n" + QString::fromStdString(create_proj_path_res.error()));
    return false;
  }

  // テンプレート用アイテムを作成
  const auto tpl_data = createTemplateData();

  // メタパッケージを作成
  if (!generateMetaPackage(tpl_data)) {
    return false;
  }

  // 設定パッケージを作成
  if (!generateConfigPackage(tpl_data)) {
    return false;
  }

  // ユーザ用Msgパッケージを作成
  if (!generateUserMsgPackage(tpl_data)) {
    return false;
  }

  // ユーザ用C++パッケージを作成
  if (!generateUserCppPackage(tpl_data)) {
    return false;
  }

  // ユーザ用Pythonパッケージを作成
  if (!generateUserPyPackage(tpl_data)) {
    return false;
  }

  // バックアップファイルを作成
  if (!generateBackupFiles()) {
    return false;
  }

  // バージョンファイルを作成
  cmn::Version version;
  version.setToCurrent();
  if (!version.save(proj_paths_.versionPath())) {
    qt::qErrorBox(parent_, "Failed to save the current version.");
    return false;
  }

  return true;
}

std::string ProjectGenerator::flightActionsPackage() const
{
  if (!settings_->controller->useBuiltinContrller()) {
    qt::qWarnBox(parent_, "You cannot use the mission planner because the builtin controller is not used.");
    return "tobas_dummy_pkg";
  }

  const auto frame_type = settings_->controller->getFrameType();

  if (
    frame_type == FrameType::kPlanarMulticopter || frame_type == FrameType::kNonPlanarMulticopter ||
    frame_type == FrameType::kYAxisTiltMulticopter || frame_type == FrameType::kRandomAxisTiltMulticopter) {
    return "tobas_multicopter_actions";
  }
  else {
    qt::qWarnBox(parent_, "Mission planner is not supported for this frame type.");
    return "tobas_dummy_pkg";
  }
}

inja::json ProjectGenerator::createTemplateData()
{
  inja::json tpl_data;

  tpl_data["drone_name"] = uadf_.urdf->getName();

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
  tpl_data["meta_pkg_name"] = proj_paths_.metaPkgName();
  tpl_data["config_pkg_name"] = proj_paths_.cfgPkgName();
  tpl_data["user_msg_pkg_name"] = proj_paths_.userMsgPkgName();
  tpl_data["user_cpp_pkg_name"] = proj_paths_.userCppPkgName();
  tpl_data["user_py_pkg_name"] = proj_paths_.userPyPkgName();

  return tpl_data;
}

tobas::Drone ProjectGenerator::createDrone()
{
  tobas::Drone drone;

  // Drone Name
  drone.name = uadf_.urdf->getName();

  // Propulsion System
  switch (settings_->propulsion_system->type()) {
    case tobas::PropulsionSystem::kElectric: {
      const auto eprop_widget =
        qt::qConstPointerCast<propulsion::electric::PropulsionSystemWidget>(settings_->propulsion_system->selected());
      const auto eprop = std::make_shared<tobas::ElectricPropulsionSystemConfig>();

      // Battery
      const auto battery_widget = eprop_widget->battery;
      eprop->battery.nominal_voltage = battery_widget->nominalVoltage();
      eprop->battery.max_voltage = battery_widget->maxVoltage();
      eprop->battery.sag_voltage = battery_widget->sagVoltage();
      eprop->battery.max_current = battery_widget->maxCurrent();
      eprop->battery.internal_resistance = battery_widget->internalRegistance();

      // Rotors
      for (int i = 0; i < eprop_widget->numUnits(); ++i) {
        const auto unit_widget = eprop_widget->units->widget(i);
        const auto link_name = eprop_widget->linkName(i).toStdString();

        const auto& cur_ele = tree_.getSegment(link_name)->second;
        const auto& cur_seg = cur_ele.segment;
        const auto& cur_jnt = cur_seg.joint();
        const auto& par_ele = cur_ele.parent->second;
        const auto& par_seg = par_ele.segment;
        const auto& par_jnt = par_seg.joint();

        // Rotor
        const auto rotor = std::make_shared<tobas::ElectricRotorConfig>();
        rotor->link_name = link_name;
        rotor->direction = turningDirectionUadfToTbsdrn(uadf_.thrusts.at(cur_jnt.name).direction);
        rotor->moment_const = unit_widget->aerodynamics()->momentConst();
        rotor->tilt_joint_name = uadf_.tilts.contains(par_jnt.name) ? par_jnt.name : "";
        rotor->channel = settings_->hardware->dshot()->channel(QString::fromStdString(cur_jnt.name));  // TODO: PWM対応
        rotor->num_poles = unit_widget->motor()->numPoles();
        rotor->kv = unit_widget->motor()->kv();
        rotor->internal_resistance = unit_widget->motor()->internalResistance();
        rotor->min_speed = unit_widget->motor()->minimumSpeed();
        rotor->propeller_diameter = unit_widget->propeller()->diameter();
        rotor->motor_const = unit_widget->aerodynamics()->motorConst();
        TOBAS_CHECK(eprop->rotors.insert({ link_name, rotor }).second);

        // Tilt Joint
        if (uadf_.tilts.contains(par_jnt.name)) {
          tobas::JointConfig tilt_joint;
          tilt_joint.name = par_jnt.name;
          tilt_joint.role = tobas::JointRole::kTiltJoint;
          tilt_joint.cmd_iface = tobas::JointCommandInterface::kPosition;
          if (settings_->hardware->pwm()->contains(QString::fromStdString(par_jnt.name))) {
            tilt_joint.hw_iface = tobas::HardwareInterface::kPwm;
          }
          else {
            tilt_joint.hw_iface = tobas::HardwareInterface::kOther;
          }
          tilt_joint.home_pos = 0.;
          TOBAS_CHECK(drone.joints.insert({ tilt_joint.name, tilt_joint }).second);

          if (tilt_joint.hw_iface == tobas::HardwareInterface::kPwm) {
            tobas::PwmConfig tilt_pwm;
            const auto pwm_channel = settings_->hardware->pwm()->channel(QString::fromStdString(par_jnt.name));
            tilt_pwm.channel = pwm_channel;
            tilt_pwm.name = par_jnt.name;
            tilt_pwm.period_range.first = settings_->hardware->pwm()->periodLb(pwm_channel);
            tilt_pwm.period_range.second = settings_->hardware->pwm()->periodUb(pwm_channel);
            tilt_pwm.value_range.first = par_jnt.lower_limit;
            tilt_pwm.value_range.second = par_jnt.upper_limit;
            TOBAS_CHECK(drone.pwms.insert({ link_name, tilt_pwm }).second);
          }
        }
      }

      drone.prop = std::static_pointer_cast<tobas::PropulsionSystemConfig>(eprop);
      break;
    }
    case tobas::PropulsionSystem::kIce: {
      const auto iprop_widget =
        qt::qConstPointerCast<propulsion::ice::PropulsionSystemWidget>(settings_->propulsion_system->selected());
      const auto iprop = std::make_shared<tobas::IcePropulsionSystemConfig>();

      // Engine
      const auto engine_widget = iprop_widget->engine;
      iprop->engine.engine_const = engine_widget->dynamics()->engineConstant();
      if (settings_->hardware->pwm()->contains(hw::PwmWidget::kEngineThrotLabel)) {
        iprop->engine.hw_iface = tobas::HardwareInterface::kPwm;
      }
      else {
        iprop->engine.hw_iface = tobas::HardwareInterface::kOther;
      }

      if (iprop->engine.hw_iface == tobas::HardwareInterface::kPwm) {
        tobas::PwmConfig engine_pwm;
        const auto engine_pwm_channel = settings_->hardware->pwm()->channel(hw::PwmWidget::kEngineThrotLabel);
        engine_pwm.channel = engine_pwm_channel;
        engine_pwm.name = tobas::pwm::kEngineThrottleKey;
        engine_pwm.period_range.first = settings_->hardware->pwm()->periodLb(engine_pwm_channel);
        engine_pwm.period_range.second = settings_->hardware->pwm()->periodUb(engine_pwm_channel);
        engine_pwm.value_range.first = tobas::kMinThrot;
        engine_pwm.value_range.second = tobas::kMaxThrot;
        TOBAS_CHECK(drone.pwms.insert({ tobas::pwm::kEngineThrottleKey, engine_pwm }).second);
      }

      // Rotors
      for (int i = 0; i < iprop_widget->numUnits(); ++i) {
        const auto unit_widget = iprop_widget->units->widget(i);
        const auto link_name = iprop_widget->linkName(i).toStdString();

        const auto& cur_ele = tree_.getSegment(link_name)->second;
        const auto& cur_seg = cur_ele.segment;
        const auto& cur_jnt = cur_seg.joint();
        const auto& par_ele = cur_ele.parent->second;
        const auto& par_seg = par_ele.segment;
        const auto& par_jnt = par_seg.joint();

        // Rotor
        const auto rotor = std::make_shared<tobas::IceRotorConfig>();
        rotor->link_name = link_name;
        rotor->direction = turningDirectionUadfToTbsdrn(uadf_.thrusts.at(cur_jnt.name).direction);
        rotor->moment_const = unit_widget->aerodynamics()->momentConst();
        rotor->tilt_joint_name = uadf_.tilts.contains(par_jnt.name) ? par_jnt.name : "";
        rotor->gear_ratio = unit_widget->transmission()->gearRatio();
        rotor->pitch_limit = unit_widget->propeller()->pitchAngleLimit();
        rotor->motor_const = unit_widget->aerodynamics()->motorConst();
        if (settings_->hardware->pwm()->contains(QString::fromStdString(cur_jnt.name))) {
          rotor->hw_iface = tobas::HardwareInterface::kPwm;
        }
        else {
          rotor->hw_iface = tobas::HardwareInterface::kOther;
        }
        TOBAS_CHECK(iprop->rotors.insert({ link_name, rotor }).second);

        // Variable Pitch Interface
        // TODO: PWM以外のインターフェースに対応
        tobas::PwmConfig pitch_pwm;
        const auto vpp_pwm_channel = settings_->hardware->pwm()->channel(QString::fromStdString(cur_jnt.name));
        pitch_pwm.channel = vpp_pwm_channel;
        pitch_pwm.name = link_name;
        pitch_pwm.period_range.first = settings_->hardware->pwm()->periodLb(vpp_pwm_channel);
        pitch_pwm.period_range.second = settings_->hardware->pwm()->periodUb(vpp_pwm_channel);
        pitch_pwm.value_range = unit_widget->propeller()->pitchAngleLimit().toPair();
        TOBAS_CHECK(drone.pwms.insert({ link_name, pitch_pwm }).second);

        // Tilt Joint
        if (uadf_.tilts.contains(par_jnt.name)) {
          tobas::JointConfig tilt_joint;
          tilt_joint.name = par_jnt.name;
          tilt_joint.role = tobas::JointRole::kTiltJoint;
          tilt_joint.cmd_iface = tobas::JointCommandInterface::kPosition;
          tilt_joint.hw_iface = tobas::HardwareInterface::kPwm;  // TODO: 選択できるようにする
          tilt_joint.home_pos = 0.;
          TOBAS_CHECK(drone.joints.insert({ tilt_joint.name, tilt_joint }).second);

          if (tilt_joint.hw_iface == tobas::HardwareInterface::kPwm) {
            tobas::PwmConfig tilt_pwm;
            const auto pwm_channel = settings_->hardware->pwm()->channel(QString::fromStdString(par_jnt.name));
            tilt_pwm.channel = pwm_channel;
            tilt_pwm.name = par_jnt.name;
            tilt_pwm.period_range.first = settings_->hardware->pwm()->periodLb(pwm_channel);
            tilt_pwm.period_range.second = settings_->hardware->pwm()->periodUb(pwm_channel);
            tilt_pwm.value_range.first = par_jnt.lower_limit;
            tilt_pwm.value_range.second = par_jnt.upper_limit;
            TOBAS_CHECK(drone.pwms.insert({ link_name, tilt_pwm }).second);
          }
        }
      }

      drone.prop = std::static_pointer_cast<tobas::PropulsionSystemConfig>(iprop);
      break;
    }
    default: {
      throw;
    }
  }

  // Fixed Wing
  if (!uadf_.control_surfaces.empty()) {
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
    const auto css = settings_->fixed_wing->controlSurfaces();
    for (int i = 0; i < css->numUnits(); ++i) {
      const auto link_name = css->linkName(i).toStdString();

      const auto& cur_ele = tree_.getSegment(link_name)->second;
      const auto& cur_seg = cur_ele.segment;
      const auto& cur_jnt = cur_seg.joint();

      tobas::ControlSurface cs;
      cs.link_name = link_name;
      cs.c_lift_delta = css->liftCoef(i);
      cs.c_drag_abs_delta = css->dragCoef(i);  // TODO: 正負の確認が必要？
      cs.c_side_delta = css->sideCoef(i);
      cs.c_roll_delta = css->rollCoef(i);
      cs.c_pitch_delta = css->pitchCoef(i);
      cs.c_yaw_delta = css->yawCoef(i);
      drone.fixed_wing->control_surfaces[link_name] = cs;

      tobas::JointConfig joint;
      joint.name = cur_jnt.name;
      joint.role = tobas::JointRole::kControlSurface;
      joint.cmd_iface = tobas::JointCommandInterface::kPosition;
      if (settings_->hardware->pwm()->contains(QString::fromStdString(cur_jnt.name))) {
        joint.hw_iface = tobas::HardwareInterface::kPwm;
      }
      else {
        joint.hw_iface = tobas::HardwareInterface::kOther;
      }
      joint.home_pos = 0.;
      TOBAS_CHECK(drone.joints.insert({ joint.name, joint }).second);
    }
  }

  // Extra Joints
  const auto& extra_joints = settings_->extra_joints;
  for (int i = 0; i < extra_joints->numJoints(); ++i) {
    tobas::JointConfig joint;
    joint.name = extra_joints->getJointName(i).toStdString();
    joint.role = extra_joints->getRole(i);
    joint.cmd_iface = extra_joints->getCommandInterface(i);
    joint.hw_iface = tobas::HardwareInterface::kOther;  // TODO: 選択できるようにする
    joint.home_pos = extra_joints->getHomePosition(i);
    TOBAS_CHECK(drone.joints.insert({ joint.name, joint }).second);
  }

  // RC Input
  drone.num_sbus_channels = settings_->rc_input->numOfSbusChannels();

  return drone;
}

bool ProjectGenerator::hasServoJoint() const
{
  if (!uadf_.tilts.empty() || !uadf_.control_surfaces.empty()) {
    return true;
  }

  const auto& extra_joints = settings_->extra_joints;
  for (int i = 0; i < extra_joints->numJoints(); ++i) {
    if (tobas::isServoJoint(extra_joints->getRole(i))) {
      return true;
    }
  }

  return false;
}

bool ProjectGenerator::generateMetaPackage(const inja::json& tpl_data)
{
  const auto meta_pkg_path = proj_paths_.metaPkgPath();
  fs::create_directory(meta_pkg_path);

  meta_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", meta_pkg_path);
  meta_env_->generate(tpl_data, "package.xml.tplxml", meta_pkg_path);

  if (!createEmptyFile(meta_pkg_path / kDoNotEditThisPackage)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateConfigPackage(const inja::json& tpl_data)
{
  const auto pkg_path = proj_paths_.cfgPkgPath();
  fs::create_directory(pkg_path);

  // ディレクトリを作成
  const auto config_dir = proj_paths_.cfgConfigDirPath();
  const auto launch_dir = proj_paths_.cfgLaunchDirPath();
  const auto mesh_dir = proj_paths_.cfgMeshDirPath();
  const auto urdf_dir = proj_paths_.cfgUrdfDirPath();
  fs::create_directory(config_dir);
  fs::create_directory(launch_dir);
  fs::create_directory(mesh_dir);
  fs::create_directory(urdf_dir);

  // テンプレートから生成
  config_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", pkg_path);
  config_env_->generate(tpl_data, "package.xml.tplxml", pkg_path);
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
  if (!createEmptyYaml(proj_paths_.imuFiltDynParamsPath(), false)) {
    return false;
  }
  if (!createEmptyYaml(proj_paths_.obsvDynParamsPath(), false)) {
    return false;
  }
  if (!createEmptyYaml(proj_paths_.ctrlDynParamsPath(), false)) {
    return false;
  }
  if (!createEmptyYaml(proj_paths_.rcTeleopDynParamsPath(), false)) {
    return false;
  }

  // その他
  if (!createEmptyFile(pkg_path / kDoNotEditThisPackage)) {
    return false;
  }
  if (!generateDroneConfig()) {
    return false;
  }
  if (!generateHealthMonitorConfig()) {
    return false;
  }
  if (!generateObserverStaticConfig()) {
    return false;
  }
  if (!generateControllerStaticConfig()) {
    return false;
  }
  if (!generateRcTeleopStaticConfig()) {
    return false;
  }
  if (!generateSshConfig()) {
    return false;
  }
  if (!generateNetworkConfig()) {
    return false;
  }
  if (!generateOriginalUadf()) {
    return false;
  }
  if (!generateModifiedUrdf()) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateUserMsgPackage(const inja::json& tpl_data)
{
  const auto pkg_path = proj_paths_.userMsgPkgPath();

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

bool ProjectGenerator::generateUserCppPackage(const inja::json& tpl_data)
{
  const auto pkg_path = proj_paths_.userCppPkgPath();

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

bool ProjectGenerator::generateUserPyPackage(const inja::json& tpl_data)
{
  const auto pkg_path = proj_paths_.userPyPkgPath();
  const auto pkg_name = proj_paths_.userPyPkgName();

  // パッケージを作成
  fs::create_directory(pkg_path);

  // 空のresourceファイルを作成
  const auto resource_file = pkg_path / "resource" / pkg_name;
  TOBAS_CHECK(path::createFilePath(resource_file, true));

  // ディレクトリを作成
  const auto launch_dir = pkg_path / "launch";
  const auto lib_dir = pkg_path / pkg_name;
  fs::create_directory(launch_dir);
  fs::create_directory(lib_dir);

  // テンプレートから作成
  user_py_env_->generate(tpl_data, "package.xml.tplxml", pkg_path, false);
  user_py_env_->generate(tpl_data, "setup.cfg.tplini", pkg_path, false);
  user_py_env_->generate(tpl_data, "setup.py.tplpy", pkg_path, false);
  user_py_env_->generate(tpl_data, "common.launch.py.tplpy", launch_dir, false);
  user_py_env_->generate(tpl_data, "real.launch.py.tplpy", launch_dir, false);
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

bool ProjectGenerator::generateBackupFiles()
{
  // ディレクトリを作成
  const auto backup_dir = proj_paths_.projBackupDirPath();
  fs::create_directory(backup_dir);

  // 設定ファイル
  const auto backup_data = settings_->dump();
  if (!saveYamlNode(proj_paths_.backupSettingsPath(), backup_data)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateDroneConfig()
{
  const auto drone = createDrone();

  const auto tbsdrn_path = proj_paths_.tbsdrnPath();
  if (!drone.save(tbsdrn_path)) {
    qt::qErrorBox(parent_, "Failed to save drone configuration.");
    return false;
  }

  return true;
}

bool ProjectGenerator::generateHealthMonitorConfig()
{
  YAML::Node node(YAML::NodeType::Map);
  node["check_realtime_compliance"] = settings_->failsafe->checkRealtimeCompliance();
  node["check_battery_voltage"] = settings_->failsafe->checkBatteryVoltage();
  node["check_cpu_temperature"] = settings_->failsafe->checkCpuTemperature();
  node["check_radio_link"] = settings_->failsafe->checkRadioLink();
  node["check_rotor_links"] = settings_->failsafe->checkRotorLinks();
  node["check_attitude_level"] = settings_->failsafe->checkAttitudeLevel();
  node["check_position_stability"] = settings_->failsafe->checkPositionStability();
  node["check_position_accuracy"] = settings_->failsafe->checkPositionAccuracy();
  node["check_velocity_accuracy"] = settings_->failsafe->checkVelocityAccuracy();
  node["check_attitude_accuracy"] = settings_->failsafe->checkAttitudeAccuracy();
  node["check_heading_accuracy"] = settings_->failsafe->checkHeadingAccuracy();
  node["check_mag_offset"] = settings_->failsafe->checkMagOffset();
  node["check_mag_alignment"] = settings_->failsafe->checkMagAlignment();
  node["check_vibration_level"] = settings_->failsafe->checkVibrationLevel();

  const auto config_dir = proj_paths_.cfgConfigDirPath();
  if (!saveYamlNode(config_dir / "health_monitor.yaml", node)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateObserverStaticConfig()
{
  YAML::Node params(YAML::NodeType::Map);
  params["frame_id"] = tree_.getRootName();
  params["use_magnetometer"] = settings_->observer->useMagnetometer();
  params["use_barometer"] = settings_->observer->useBarometer();
  params["use_gnss"] = settings_->observer->useGnss();
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

  const auto config_dir = proj_paths_.cfgConfigDirPath();

  // For component
  const auto node_component = params;
  if (!saveYamlNode(config_dir / "observer_static.yaml", node_component)) {
    return false;
  }

  // For standalone
  YAML::Node node_standalone(YAML::NodeType::Map);
  node_standalone["/**"][tobas::node::kObserver][kRosParamsKey] = params;
  if (!saveYamlNode(config_dir / "observer_static_standalone.yaml", node_standalone)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateControllerStaticConfig()
{
  const auto params = settings_->controller->staticParams();
  TOBAS_CHECK(params.IsMap());

  const auto config_dir = proj_paths_.cfgConfigDirPath();

  // For component
  const auto node_component = params;
  if (!saveYamlNode(config_dir / "controller_static.yaml", node_component)) {
    return false;
  }

  // For standalone
  YAML::Node node_standalone(YAML::NodeType::Map);
  node_standalone["/**"][tobas::node::kController][kRosParamsKey] = params;
  if (!saveYamlNode(config_dir / "controller_static_standalone.yaml", node_standalone)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateRcTeleopStaticConfig()
{
  YAML::Node params(YAML::NodeType::Map);
  params["acrobat_mode"] = settings_->controller->acrobatModeCommand();
  params["stabilize_mode"] = settings_->controller->stabilizeModeCommand();
  params["loiter_mode"] = settings_->controller->loiterModeCommand();

  const auto config_dir = proj_paths_.cfgConfigDirPath();

  // For component
  const auto node_component = params;
  if (!saveYamlNode(config_dir / "rc_teleop_static.yaml", node_component)) {
    return false;
  }

  // For standalone
  YAML::Node node_standalone(YAML::NodeType::Map);
  node_standalone["/**"][tobas::node::kRcTeleop][kRosParamsKey] = params;
  if (!saveYamlNode(config_dir / "rc_teleop_static_standalone.yaml", node_standalone)) {
    return false;
  }

  return true;
}

bool ProjectGenerator::generateSshConfig()
{
  cmn::SshConfig config;
  config.host = settings_->remote_connection->host().toStdString();
  config.user = tobas::kFmuUserName;

  if (!config.save(proj_paths_.sshConfigPath())) {
    qt::qErrorBox(parent_, "Failed to save the SSH configurations.");
    return false;
  }

  return true;
}

bool ProjectGenerator::generateNetworkConfig()
{
  cmn::NetworkConfig config;
  config.interface = settings_->network->networkInterfaceName().toStdString();

  if (!config.save(proj_paths_.networkConfigPath())) {
    qt::qErrorBox(parent_, "Failed to save the network configurations.");
    return false;
  }

  return true;
}

bool ProjectGenerator::generateOriginalUadf()
{
  // Export the original UADF
  const auto doc = uadf::exportUADF(uadf_);
  const auto robot = doc->RootElement();

  // Modify
  if (!replaceOriginalUadfMeshFilePaths(robot)) {
    return false;
  }

  // Save
  if (doc->SaveFile(proj_paths_.originalUadfPath().c_str()) != tinyxml2::XML_SUCCESS) {
    qt::qErrorBox(parent_, "Failed to save the original UADF.");
    return false;
  }

  return true;
}

bool ProjectGenerator::generateModifiedUrdf()
{
  // Export the original URDF
  const auto doc = urdf::exportUrdf(*uadf_.urdf);
  const auto robot = doc->RootElement();

  // Modify
  if (!resolveModifiedUrdfMeshFilePaths(robot)) {
    return false;
  }
  if (!removePropellerJointLimits(robot)) {
    return false;
  }
  if (!addXmlElements(robot)) {
    return false;
  }

  // Save
  if (doc->SaveFile(proj_paths_.xacroPath().c_str()) != tinyxml2::XML_SUCCESS) {
    qt::qErrorBox(parent_, "Failed to save the modified URDF.");
    return false;
  }

  return true;
}

bool ProjectGenerator::createEmptyFile(const fs::path& file_path)
{
  const auto res = path::createFilePath(file_path, true);
  if (!res) {
    qt::qErrorBox(parent_, "Failed to create \"" + QString::fromStdString(file_path) + "\":\n" + res.error().c_str());
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
    qt::qErrorBox(parent_, "Failed to save \"" + QString::fromStdString(path) + "\".");
    return false;
  }

  return true;
}

bool ProjectGenerator::resolveModifiedUrdfMeshFilePaths(tinyxml2::XMLElement* elem)
{
  if (strcmp(elem->Name(), "mesh") == 0) {
    const auto filename = elem->Attribute("filename");
    if (!filename) {
      qt::qErrorBox(parent_, "Mesh element does not have attribute: \"filename\"");
      return false;
    }

    const auto src_path = urdf::resolveURI(filename);
    if (!fs::exists(src_path)) {
      qt::qErrorBox(parent_, "Mesh file " + QString::fromStdString(src_path) + " does not exist.");
      return false;
    }

    // 必要に応じてメッシュファイルをTobasパッケージ以下にコピー
    const auto mesh_dir = proj_paths_.cfgMeshDirPath();
    const auto base_name = src_path.filename();
    const auto dst_path = mesh_dir / base_name;
    if (fs::exists(dst_path)) {
      // dst_pathが存在するがsrc_pathと内容が異なる場合は，fs::copy_fileでは上書きされないため一度削除した上でコピーする．
      if (!fs::equivalent(src_path, dst_path)) {
        if (!fs::remove(dst_path)) {
          qt::qErrorBox(parent_, "Failed to remove " + QString::fromStdString(dst_path) + ".");
          return false;
        }

        if (!fs::copy_file(src_path, dst_path)) {
          qt::qErrorBox(
            parent_,
            "Failed to copy " + QString::fromStdString(src_path) + " to " + QString::fromStdString(dst_path) + ".");
          return false;
        }
      }
    }
    else {
      // dst_pathが存在しない場合は，ただコピーすればよい．
      if (!fs::copy_file(src_path, dst_path)) {
        qt::qErrorBox(
          parent_,
          "Failed to copy " + QString::fromStdString(src_path) + " to " + QString::fromStdString(dst_path) + ".");
        return false;
      }
    }

    // メッシュファイルへのパスを置換
    // package://<pkg_name>の書式だとIgnitionが発見できないため，絶対パスに置換できるようxacroコマンドを埋め込む．
    // cf. https://github.com/moveit/moveit_resources/blob/ros2/panda_description/urdf/panda.urdf.xacro
    const auto cfg_pkg_name = proj_paths_.cfgPkgName();
    const auto new_filename = "file://$(find " + cfg_pkg_name + ")/meshes/" + base_name.string();
    elem->SetAttribute("filename", new_filename.c_str());
  }

  // 再帰的に子要素もチェック
  for (auto child = elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (!resolveModifiedUrdfMeshFilePaths(child)) {
      return false;
    }
  }

  return true;
}

bool ProjectGenerator::replaceOriginalUadfMeshFilePaths(tinyxml2::XMLElement* elem)
{
  if (strcmp(elem->Name(), "mesh") == 0) {
    const auto filename = elem->Attribute("filename");
    if (!filename) {
      qt::qErrorBox(parent_, "Mesh element does not have attribute: \"filename\"");
      return false;
    }

    const auto src_path = urdf::resolveURI(filename);
    const auto base_name = src_path.filename().string();
    const auto cfg_pkg_name = proj_paths_.cfgPkgName();

    // config_pkgからの相対パスで指定
    const auto new_filename = "package://" + cfg_pkg_name + "/meshes/" + base_name;
    elem->SetAttribute("filename", new_filename.c_str());
  }

  // 再帰的に子要素もチェック
  for (auto child = elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (!replaceOriginalUadfMeshFilePaths(child)) {
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
    const auto jnt_name = tree_.getSegment(link_name)->second.segment.joint().name;
    prop_jnt_names.insert(jnt_name);
  }

  for (auto child = robot->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (strcmp(child->Name(), "joint") == 0) {
      const auto jnt_name = child->Attribute("name");
      if (!jnt_name) {
        qt::qErrorBox(parent_, "Joint element does not have attribute: \"name\"");
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

bool ProjectGenerator::addXmlElements(tinyxml2::XMLElement* robot)
{
  const auto& ns = uadf_.urdf->getName();
  const auto& root_name = tree_.getRootName();
  const auto cfg_pkg_name = proj_paths_.cfgPkgName();

  const auto& prop = settings_->propulsion_system;
  const auto& fmu = settings_->hardware;

  const auto drone = createDrone();

  // Get rotor link names
  std::vector<std::string> rotor_link_names;
  for (const auto& [link_name, _] : drone.prop->rotors) {
    rotor_link_names.push_back(link_name);
  }

  // XML namespace
  robot->SetAttribute("xmlns:xacro", "http://ros.org/wiki/xacro");

  // IMU plugin
  xml::addImuPlugin(
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
    fmu->magNoiseStddev(),
    fmu->magHardBiasNorm());

  // Barometer plugin
  xml::addBarometerPlugin(
    robot,
    ns,
    root_name,
    fmu->presUpdateRate(),
    Eigen::Vector3d::Zero(),  // TODO
    fmu->presNoiseStddev());

  // GNSS plugin
  xml::addGnssPlugin(
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
    fmu->gnssVerticalVelocityStddev());

  // Propulsion system plugins
  switch (drone.prop->type()) {
    case tobas::PropulsionSystem::kElectric: {
      const auto eprop = qt::qConstPointerCast<propulsion::electric::PropulsionSystemWidget>(prop->selected());
      const auto battery = eprop->battery;
      const auto units = eprop->units;

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
        const auto& cur_ele = tree_.getSegment(link_name)->second;
        const auto& cur_seg = cur_ele.segment;
        const auto& cur_jnt = cur_seg.joint();

        const auto unit = units->widget(i);
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
          propeller->numBlades(),
          aero->motorConst(),
          aero->momentConst(),
          aero->dragConst(),
          turningDirectionUadfToTbsdrn(uadf_.thrusts.at(cur_jnt.name).direction),
          esc->maxCurrent());
      }

      break;
    }
    case tobas::PropulsionSystem::kIce: {
      const auto iprop = qt::qConstPointerCast<propulsion::ice::PropulsionSystemWidget>(prop->selected());
      const auto engine = iprop->engine;
      const auto units = iprop->units;

      xml::EngineParam engine_param;
      engine_param.engine_const = engine->dynamics()->engineConstant();
      engine_param.time_const_up = engine->response()->timeConstUp();
      engine_param.time_const_down = engine->response()->timeConstDown();

      std::vector<xml::IceRotorParam> rotor_params;
      for (int i = 0; i < units->numUnits(); ++i) {
        const auto link_name = iprop->linkName(i).toStdString();
        const auto& cur_ele = tree_.getSegment(link_name)->second;
        const auto& cur_seg = cur_ele.segment;
        const auto& cur_jnt = cur_seg.joint();

        const auto unit = units->widget(i);

        xml::IceRotorParam rotor_param;
        rotor_param.link_name = iprop->linkName(i).toStdString();
        rotor_param.direction = turningDirectionUadfToTbsdrn(uadf_.thrusts.at(cur_jnt.name).direction);
        rotor_param.gear_ratio = unit->transmission()->gearRatio();
        rotor_param.num_blades = unit->propeller()->numBlades();
        rotor_param.pitch_angle_limit = unit->propeller()->pitchAngleLimit();
        rotor_param.max_pitch_angle_rate = unit->propeller()->maxPitchAngleRate();
        rotor_param.motor_const = unit->aerodynamics()->motorConst();
        rotor_param.moment_const = unit->aerodynamics()->momentConst();
        rotor_param.drag_const = unit->aerodynamics()->dragConst();

        rotor_params.push_back(rotor_param);
      }

      xml::addIcePropulsionSystemPlugin(robot, ns, engine_param, rotor_params);

      break;
    }
    default: {
      throw;
    }
  }

  // Fixed wing plugin
  if (drone.fixed_wing) {
    xml::addFixedWingPlugin(robot, ns, root_name, *drone.fixed_wing);
  }

  // Joint state broadcaster plugin
  std::vector<std::string> joint_names;
  for (const auto& [jnt_name, _] : drone.joints) {
    joint_names.push_back(jnt_name);
  }
  xml::addJointStateBroadcasterPlugin(robot, ns, joint_names, 100);  // TODO: GUIで更新レートを調整できるように

  // Joint controller plugins
  for (const auto& [_, joint] : drone.joints) {
    if (!joint.isServoJoint()) {
      continue;
    }

    switch (joint.cmd_iface) {
      case tobas::JointCommandInterface::kNone: {
        break;
      }
      case tobas::JointCommandInterface::kPosition: {
        const auto max_vel = uadf_.urdf->getJoint(joint.name)->limits->velocity;
        if (max_vel <= 0.) {
          qWarning() << "The maximum velocity of " << QString::fromStdString(joint.name) << " is invalid: " << max_vel;
          break;
        }

        // 最大速度で60deg回転にかかる時間を時定数とする．つまり誤差60degで最大速度が出る．
        // TODO: サーボモータの仕様 (無付加回転数など) をより正確に再現
        const auto time_const = M_PI_3 / max_vel;
        xml::addJointPositionControllerPlugin(robot, ns, joint.name, joint.home_pos, time_const);
        break;
      }
      case tobas::JointCommandInterface::kVelocity: {
        xml::addJointVelocityControllerPlugin(robot, ns, joint.name, joint.home_pos);
        break;
      }
      case tobas::JointCommandInterface::kEffort: {
        xml::addJointEffortControllerPlugin(robot, ns, joint.name, joint.home_pos);
        break;
      }
      default: {
        throw;
      }
    }
  }

  // Wind plugin
  xml::addGazeboWindPlugin(robot, ns, root_name);

  // Suspended load plugin
  xml::addGazeboSuspendedLoadPlugin(robot, ns, root_name);

  // Ground truth state plugin
  xml::addGazeboGroundTruthStatePlugin(robot, ns, root_name);

  // LookAt position plugin
  xml::addGazeboLookAtPositionPlugin(robot, ns, root_name);

  // Base static joint for debug
  xml::addBaseStaticJoint(robot, tree_.getRootName());

  return true;
}

tobas::TurningDirection ProjectGenerator::turningDirectionUadfToTbsdrn(const uadf::Thrust::Direction& src)
{
  switch (src) {
    case uadf::Thrust::CW:
      return tobas::TurningDirection::CW;
    case uadf::Thrust::CCW:
      return tobas::TurningDirection::CCW;
    default:
      throw;
  }
}
}  // namespace sa
}  // namespace gui
