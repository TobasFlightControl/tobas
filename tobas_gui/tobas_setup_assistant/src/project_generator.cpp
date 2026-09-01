// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/project_generator.hpp"

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <tobas_constants/imu.hpp>
#include <tobas_constants/node.hpp>
#include <tobas_constants/pwm_key.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_gui_common/network_config.hpp>
#include <tobas_gui_common/project_paths.hpp>
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
#include <tobas_yaml_tools/format.hpp>

#include "tobas_setup_assistant/util.hpp"
#include "tobas_setup_assistant/xml_elements/xml_elements.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace
{
constexpr char kRosParamsKey[] = "ros__parameters";

constexpr char kDoNotEditThisPackage[] = "DO_NOT_EDIT_THIS_PACKAGE";
constexpr char kYouCanEditThisPackage[] = "YOU_CAN_EDIT_THIS_PACKAGE";

TurningDirection turningDirectionUadfToTbsdrn(const uadf::Thrust::Direction& src)
{
  switch (src) {
    case uadf::Thrust::CW:
      return TurningDirection::CW;
    case uadf::Thrust::CCW:
      return TurningDirection::CCW;
    default:
      throw;
  }
}
}  // namespace

ProjectGenerator::ProjectGenerator(
  const uadf::Model& uadf,
  const kdl::Tree& tree,
  const SettingsWidget* settings,
  QWidget* parent)
  : uadf_(uadf), tree_(tree), settings_(settings), parent_(parent)
{
  const auto pkg_share_path = QString::fromStdString(getPkgShareDir().string());
  const auto templates_path = QDir(pkg_share_path).filePath("templates");
  meta_env_.emplace(QDir(templates_path).filePath("meta_package"));
  config_env_.emplace(QDir(templates_path).filePath("config_package"));
  user_msg_env_.emplace(QDir(templates_path).filePath("user_msg_package"));
  user_cpp_env_.emplace(QDir(templates_path).filePath("user_cpp_package"));
  user_py_env_.emplace(QDir(templates_path).filePath("user_py_package"));
}

void ProjectGenerator::generateProject(const QString& proj_path)
{
  // Set the project path.
  proj_paths_.setProjPath(proj_path);

  // Create the Tobas package.
  TOBAS_CHECK(QDir().mkpath(proj_path));

  // Create template items.
  const auto tpl_data = createTemplateData();

  // Create the meta package.
  generateMetaPackage(tpl_data);

  // Create the configuration package.
  generateConfigPackage(tpl_data);

  // Create the user message package.
  generateUserMsgPackage(tpl_data);

  // Create the user C++ package.
  generateUserCppPackage(tpl_data);

  // Create the user Python package.
  generateUserPyPackage(tpl_data);

  // Create the backup file.
  generateBackupFiles();

  // Create the version file.
  TOBAS_CHECK(cmn::Version::Current().save(proj_paths_.versionPath()));
}

void ProjectGenerator::setClearDynamicParams(bool flag)
{
  config_.clear_dynamic_params = flag;
}

inja::json ProjectGenerator::createTemplateData() const
{
  inja::json tpl_data;

  tpl_data["drone_name"] = uadf_.urdf->getName();

  // Controller
  tpl_data["controller_pkg"] = settings_->controller->controllerPackage().toStdString();
  tpl_data["controller_plugin"] = settings_->controller->pluginName().toStdString();

  // Mission Executor
  tpl_data["mission_executor_pkg"] = settings_->mission->executorPackage().toStdString();
  tpl_data["mission_executor_plugin"] = settings_->mission->pluginName().toStdString();

  // Hardware
  tpl_data["hardware_pkg"] = settings_->hardware->hardwarePackage();

  // Author Info
  tpl_data["author_name"] = settings_->author_info->authorName().toStdString();
  tpl_data["author_email"] = settings_->author_info->authorEmail().toStdString();

  // Ros Package
  tpl_data["meta_pkg_name"] = proj_paths_.metaPkgName().toStdString();
  tpl_data["config_pkg_name"] = proj_paths_.cfgPkgName().toStdString();
  tpl_data["user_msg_pkg_name"] = proj_paths_.userMsgPkgName().toStdString();
  tpl_data["user_cpp_pkg_name"] = proj_paths_.userCppPkgName().toStdString();
  tpl_data["user_py_pkg_name"] = proj_paths_.userPyPkgName().toStdString();

  return tpl_data;
}

Drone ProjectGenerator::createDrone() const
{
  Drone drone;

  // Drone Name
  drone.name = uadf_.urdf->getName();

  // Propulsion System
  switch (settings_->propulsion_system->type()) {
    case PropulsionSystem::kElectric: {
      const auto eprop_widget =
        qt::qConstPointerCast<propulsion::electric::PropulsionSystemWidget>(settings_->propulsion_system->selected());
      const auto eprop = std::make_shared<ElectricPropulsionSystemConfig>();

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
        const auto rotor = std::make_shared<ElectricRotorConfig>();
        rotor->link_name = link_name;
        rotor->direction = turningDirectionUadfToTbsdrn(uadf_.thrusts.at(cur_jnt.name).direction);
        rotor->moment_const = unit_widget->aerodynamics()->momentConst();
        rotor->tilt_joint_name = uadf_.tilts.contains(par_jnt.name) ? par_jnt.name : "";
        rotor->channel = settings_->hardware->dshot()->channel(QString::fromStdString(cur_jnt.name));
        rotor->num_poles = unit_widget->motor()->numPoles();
        rotor->kv = unit_widget->motor()->kv();
        rotor->internal_resistance = unit_widget->motor()->internalResistance();
        rotor->min_speed = unit_widget->motor()->minimumSpeed();
        rotor->propeller_diameter = unit_widget->propeller()->diameter();
        rotor->motor_const = unit_widget->aerodynamics()->motorConst();
        TOBAS_CHECK(eprop->rotors.insert({ link_name, rotor }).second);

        // Tilt Joint
        if (uadf_.tilts.contains(par_jnt.name)) {
          JointConfig tilt_joint;
          tilt_joint.name = par_jnt.name;
          tilt_joint.role = JointRole::kTiltJoint;
          tilt_joint.cmd_iface = JointCommandInterface::kPosition;
          if (settings_->hardware->pwm()->contains(QString::fromStdString(par_jnt.name))) {
            tilt_joint.hw_iface = HardwareInterface::kPwm;
          }
          else {
            tilt_joint.hw_iface = HardwareInterface::kOther;
          }
          tilt_joint.home_pos = 0.0;
          TOBAS_CHECK(drone.joints.insert({ tilt_joint.name, tilt_joint }).second);

          if (tilt_joint.hw_iface == HardwareInterface::kPwm) {
            PwmConfig tilt_pwm;
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

      drone.prop = std::static_pointer_cast<PropulsionSystemConfig>(eprop);
      break;
    }
    case PropulsionSystem::kIce: {
      const auto iprop_widget =
        qt::qConstPointerCast<propulsion::ice::PropulsionSystemWidget>(settings_->propulsion_system->selected());
      const auto iprop = std::make_shared<IcePropulsionSystemConfig>();

      // Engine
      const auto engine_widget = iprop_widget->engine;
      iprop->engine.engine_const = engine_widget->dynamics()->engineConstant();
      if (settings_->hardware->pwm()->contains(hw::PwmWidget::kEngineThrotLabel)) {
        iprop->engine.hw_iface = HardwareInterface::kPwm;
      }
      else {
        iprop->engine.hw_iface = HardwareInterface::kOther;
      }

      if (iprop->engine.hw_iface == HardwareInterface::kPwm) {
        PwmConfig engine_pwm;
        const auto engine_pwm_channel = settings_->hardware->pwm()->channel(hw::PwmWidget::kEngineThrotLabel);
        engine_pwm.channel = engine_pwm_channel;
        engine_pwm.name = pwm_key::kEngineThrottleKey;
        engine_pwm.period_range.first = settings_->hardware->pwm()->periodLb(engine_pwm_channel);
        engine_pwm.period_range.second = settings_->hardware->pwm()->periodUb(engine_pwm_channel);
        engine_pwm.value_range.first = kMinThrot;
        engine_pwm.value_range.second = kMaxThrot;
        TOBAS_CHECK(drone.pwms.insert({ pwm_key::kEngineThrottleKey, engine_pwm }).second);
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
        const auto rotor = std::make_shared<IceRotorConfig>();
        rotor->link_name = link_name;
        rotor->direction = turningDirectionUadfToTbsdrn(uadf_.thrusts.at(cur_jnt.name).direction);
        rotor->moment_const = unit_widget->aerodynamics()->momentConst();
        rotor->tilt_joint_name = uadf_.tilts.contains(par_jnt.name) ? par_jnt.name : "";
        rotor->gear_ratio = unit_widget->transmission()->gearRatio();
        rotor->pitch_limit = unit_widget->propeller()->pitchAngleLimit();
        rotor->center_pitch = unit_widget->propeller()->centerPitchAngle();
        rotor->motor_const = unit_widget->aerodynamics()->motorConst();
        if (settings_->hardware->pwm()->contains(QString::fromStdString(cur_jnt.name))) {
          rotor->hw_iface = HardwareInterface::kPwm;
        }
        else {
          rotor->hw_iface = HardwareInterface::kOther;
        }
        TOBAS_CHECK(iprop->rotors.insert({ link_name, rotor }).second);

        // Variable Pitch Interface
        // TODO: Support interfaces other than PWM.
        PwmConfig pitch_pwm;
        const auto vpp_pwm_channel = settings_->hardware->pwm()->channel(QString::fromStdString(cur_jnt.name));
        pitch_pwm.channel = vpp_pwm_channel;
        pitch_pwm.name = link_name;
        pitch_pwm.period_range.first = settings_->hardware->pwm()->periodLb(vpp_pwm_channel);
        pitch_pwm.period_range.second = settings_->hardware->pwm()->periodUb(vpp_pwm_channel);
        pitch_pwm.value_range = unit_widget->propeller()->pitchAngleLimit().toPair();
        TOBAS_CHECK(drone.pwms.insert({ link_name, pitch_pwm }).second);

        // Tilt Joint
        if (uadf_.tilts.contains(par_jnt.name)) {
          JointConfig tilt_joint;
          tilt_joint.name = par_jnt.name;
          tilt_joint.role = JointRole::kTiltJoint;
          tilt_joint.cmd_iface = JointCommandInterface::kPosition;
          tilt_joint.hw_iface = HardwareInterface::kPwm;  // TODO: Make this selectable.
          tilt_joint.home_pos = 0.0;
          TOBAS_CHECK(drone.joints.insert({ tilt_joint.name, tilt_joint }).second);

          if (tilt_joint.hw_iface == HardwareInterface::kPwm) {
            PwmConfig tilt_pwm;
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

      drone.prop = std::static_pointer_cast<PropulsionSystemConfig>(iprop);
      break;
    }
    default: {
      throw;
    }
  }

  // Fixed Wing
  if (!uadf_.control_surfaces.empty()) {
    drone.fixed_wing = std::make_shared<FixedWingConfig>();

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

      ControlSurface cs;
      cs.link_name = link_name;
      cs.c_lift_delta = css->liftCoef(i);
      cs.c_drag_abs_delta = css->dragCoef(i);  // TODO: Do the signs need to be checked?
      cs.c_side_delta = css->sideCoef(i);
      cs.c_roll_delta = css->rollCoef(i);
      cs.c_pitch_delta = css->pitchCoef(i);
      cs.c_yaw_delta = css->yawCoef(i);
      drone.fixed_wing->control_surfaces[link_name] = cs;

      JointConfig joint;
      joint.name = cur_jnt.name;
      joint.role = JointRole::kControlSurface;
      joint.cmd_iface = JointCommandInterface::kPosition;
      if (settings_->hardware->pwm()->contains(QString::fromStdString(cur_jnt.name))) {
        joint.hw_iface = HardwareInterface::kPwm;
      }
      else {
        joint.hw_iface = HardwareInterface::kOther;
      }
      joint.home_pos = 0.0;
      TOBAS_CHECK(drone.joints.insert({ joint.name, joint }).second);
    }
  }

  // Extra Joints
  const auto& extra_joints = settings_->extra_joints;
  for (int i = 0; i < extra_joints->numJoints(); ++i) {
    JointConfig joint;
    joint.name = extra_joints->getJointName(i).toStdString();
    joint.role = extra_joints->getRole(i);
    joint.cmd_iface = extra_joints->getCommandInterface(i);
    joint.hw_iface = HardwareInterface::kOther;  // TODO: Make this selectable.
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
    if (isServoJoint(extra_joints->getRole(i))) {
      return true;
    }
  }

  return false;
}

void ProjectGenerator::generateMetaPackage(const inja::json& tpl_data)
{
  const auto meta_pkg_path = proj_paths_.metaPkgPath();
  TOBAS_CHECK(QDir().mkpath(meta_pkg_path));

  meta_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", meta_pkg_path);
  meta_env_->generate(tpl_data, "package.xml.tplxml", meta_pkg_path);

  createEmptyFile(QDir(meta_pkg_path).filePath(kDoNotEditThisPackage));
}

void ProjectGenerator::generateConfigPackage(const inja::json& tpl_data)
{
  const auto pkg_path = proj_paths_.cfgPkgPath();
  TOBAS_CHECK(QDir().mkpath(pkg_path));

  // Create directories.
  const auto config_dir = proj_paths_.cfgConfigDirPath();
  const auto launch_dir = proj_paths_.cfgLaunchDirPath();
  const auto mesh_dir = proj_paths_.cfgMeshDirPath();
  const auto urdf_dir = proj_paths_.cfgUrdfDirPath();
  TOBAS_CHECK(QDir().mkpath(config_dir));
  TOBAS_CHECK(QDir().mkpath(launch_dir));
  TOBAS_CHECK(QDir().mkpath(mesh_dir));
  TOBAS_CHECK(QDir().mkpath(urdf_dir));

  // Generate from templates.
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

  // Dynamic parameters
  createEmptyYaml(proj_paths_.imuFiltDynParamsPath(), config_.clear_dynamic_params);
  createEmptyYaml(proj_paths_.rpmCtrlDynParamsPath(), config_.clear_dynamic_params);
  createEmptyYaml(proj_paths_.obsvDynParamsPath(), config_.clear_dynamic_params);
  createEmptyYaml(proj_paths_.ctrlDynParamsPath(), config_.clear_dynamic_params);
  createEmptyYaml(proj_paths_.rcTeleopDynParamsPath(), config_.clear_dynamic_params);

  // Other files.
  createEmptyFile(QDir(pkg_path).filePath(kDoNotEditThisPackage));
  generateDroneConfig();
  generateHealthMonitorConfig();
  generateObserverStaticConfig();
  generateControllerStaticConfig();
  generateMissionExecutorStaticConfig();
  generateRcTeleopStaticConfig();
  generateImuFilterConfig();
  generateRotorAnomalyDetectorConfig();
  generateNetworkConfig();
  generateOriginalUadf();
  generateModifiedUrdf();
}

void ProjectGenerator::generateUserMsgPackage(const inja::json& tpl_data)
{
  const auto pkg_path = proj_paths_.userMsgPkgPath();

  // Create the package.
  TOBAS_CHECK(QDir().mkpath(pkg_path));

  // Create from templates.
  user_msg_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", pkg_path, false);
  user_msg_env_->generate(tpl_data, "package.xml.tplxml", pkg_path, false);

  // Other files.
  createEmptyFile(QDir(pkg_path).filePath(kYouCanEditThisPackage));
}

void ProjectGenerator::generateUserCppPackage(const inja::json& tpl_data)
{
  const auto pkg_path = proj_paths_.userCppPkgPath();

  // Create the package.
  TOBAS_CHECK(QDir().mkpath(pkg_path));

  // Create directories.
  const auto launch_dir = QDir(pkg_path).filePath("launch");
  const auto nodes_dir = QDir(pkg_path).filePath("nodes");
  TOBAS_CHECK(QDir().mkpath(launch_dir));
  TOBAS_CHECK(QDir().mkpath(nodes_dir))

  // Create from templates.
  user_cpp_env_->generate(tpl_data, "CMakeLists.txt.tplcmake", pkg_path, false);
  user_cpp_env_->generate(tpl_data, "package.xml.tplxml", pkg_path, false);
  user_cpp_env_->generate(tpl_data, "common_realtime.launch.py.tplpy", launch_dir, false);
  user_cpp_env_->generate(tpl_data, "common_interface.launch.py.tplpy", launch_dir, false);
  user_cpp_env_->generate(tpl_data, "real_realtime.launch.py.tplpy", launch_dir, false);
  user_cpp_env_->generate(tpl_data, "real_interface.launch.py.tplpy", launch_dir, false);
  user_cpp_env_->generate(tpl_data, "gazebo.launch.py.tplpy", launch_dir, false);
  user_cpp_env_->generate(tpl_data, "user_node.cpp.tplcpp", nodes_dir, false);

  // Other files.
  createEmptyFile(QDir(pkg_path).filePath(kYouCanEditThisPackage));
}

void ProjectGenerator::generateUserPyPackage(const inja::json& tpl_data)
{
  const auto pkg_path = proj_paths_.userPyPkgPath();
  const auto pkg_name = proj_paths_.userPyPkgName();

  // Create the package.
  TOBAS_CHECK(QDir().mkpath(pkg_path));

  // Create an empty resource file.
  const auto resource_file = QDir(pkg_path).filePath("resource/" + pkg_name);
  TOBAS_CHECK(path::createFilePath(resource_file.toStdString(), true));

  // Create directories.
  const auto launch_dir = QDir(pkg_path).filePath("launch");
  const auto lib_dir = QDir(pkg_path).filePath(pkg_name);
  TOBAS_CHECK(QDir().mkpath(launch_dir));
  TOBAS_CHECK(QDir().mkpath(lib_dir));

  // Create from templates.
  user_py_env_->generate(tpl_data, "package.xml.tplxml", pkg_path, false);
  user_py_env_->generate(tpl_data, "setup.cfg.tplini", pkg_path, false);
  user_py_env_->generate(tpl_data, "setup.py.tplpy", pkg_path, false);
  user_py_env_->generate(tpl_data, "common.launch.py.tplpy", launch_dir, false);
  user_py_env_->generate(tpl_data, "real.launch.py.tplpy", launch_dir, false);
  user_py_env_->generate(tpl_data, "gazebo.launch.py.tplpy", launch_dir, false);
  user_py_env_->generate(tpl_data, "user_node.py.tplpy", lib_dir, false);

  // Other files.
  createEmptyFile(QDir(pkg_path).filePath(kYouCanEditThisPackage));
  createEmptyFile(QDir(lib_dir).filePath("__init__.py"));
}

void ProjectGenerator::generateBackupFiles()
{
  // Create directories.
  const auto backup_dir = proj_paths_.projBackupDirPath();
  TOBAS_CHECK(QDir().mkpath(backup_dir));

  // Configuration files.
  const auto backup_data = settings_->dump();
  saveYamlNode(proj_paths_.backupSettingsPath(), backup_data);
}

void ProjectGenerator::generateDroneConfig()
{
  const auto drone = createDrone();
  const auto tbsdrn_path = proj_paths_.tbsdrnPath();
  TOBAS_CHECK(drone.save(tbsdrn_path.toStdString()));
}

void ProjectGenerator::generateHealthMonitorConfig()
{
  YAML::Node params(YAML::NodeType::Map);
  params["check_realtime_compliance"] = settings_->failsafe->checkRealtimeCompliance();
  params["check_battery_voltage"] = settings_->failsafe->checkBatteryVoltage();
  params["check_cpu_temperature"] = settings_->failsafe->checkCpuTemperature();
  params["check_radio_link"] = settings_->failsafe->checkRadioLink();
  params["check_rotor_links"] = settings_->failsafe->checkRotorLinks();
  params["check_attitude_level"] = settings_->failsafe->checkAttitudeLevel();
  params["check_gnss_fix"] = settings_->failsafe->checkGnssFix();
  params["check_position_stability"] = settings_->failsafe->checkPositionStability();
  params["check_horizontal_position_accuracy"] = settings_->failsafe->checkHorizontalPositionAccuracy();
  params["check_vertical_position_accuracy"] = settings_->failsafe->checkVerticalPositionAccuracy();
  params["check_attitude_accuracy"] = settings_->failsafe->checkAttitudeAccuracy();
  params["check_heading_accuracy"] = settings_->failsafe->checkHeadingAccuracy();
  params["check_mag_offset"] = settings_->failsafe->checkMagOffset();
  params["check_mag_alignment"] = settings_->failsafe->checkMagAlignment();
  params["check_vibration_level"] = settings_->failsafe->checkVibrationLevel();
  params["check_user_defined_condition"] = settings_->failsafe->checkUserDefinedCondition();

  const auto config_dir = proj_paths_.cfgConfigDirPath();

  // For component
  const auto node_component = params;
  saveYamlNode(QDir(config_dir).filePath("health_monitor.yaml"), node_component);

  // For standalone
  YAML::Node node_standalone(YAML::NodeType::Map);
  node_standalone["/**"]["health_monitor"][kRosParamsKey] = params;
  saveYamlNode(QDir(config_dir).filePath("health_monitor_standalone.yaml"), node_standalone);
}

void ProjectGenerator::generateRotorAnomalyDetectorConfig()
{
  YAML::Node params(YAML::NodeType::Map);
  params["no_communication_timeout"] = yaml::format(settings_->failsafe->escNoCommunicationTimeout());

  const auto config_dir = proj_paths_.cfgConfigDirPath();

  // For component
  const auto node_component = params;
  saveYamlNode(QDir(config_dir).filePath("rotor_anomaly_detector.yaml"), node_component);

  // For standalone
  YAML::Node node_standalone(YAML::NodeType::Map);
  node_standalone["/**"]["rotor_anomaly_detector"][kRosParamsKey] = params;
  saveYamlNode(QDir(config_dir).filePath("rotor_anomaly_detector_standalone.yaml"), node_standalone);
}

void ProjectGenerator::generateObserverStaticConfig()
{
  YAML::Node params(YAML::NodeType::Map);
  params["frame_id"] = tree_.getRootName();
  params["use_magnetometer"] = settings_->observer->useMagnetometer();
  params["use_barometer"] = settings_->observer->useBarometer();
  params["use_gnss"] = settings_->observer->useGnss();
  params["use_external_pose"] = settings_->observer->useExternalPose();
  params["adaptive_gnss_noise"] = settings_->observer->adaptiveGnssNoise();
  params["adaptive_grav_noise"] = settings_->observer->adaptiveGravityNoise();
  params["do_acc_bias_estimation"] = settings_->observer->doAccelBiasEstimation();
  params["do_gyro_bias_estimation"] = settings_->observer->doGyroBiasEstimation();
  params["do_mag_hard_bias_estimation"] = settings_->observer->doMagHardBiasEstimation();
  params["do_mag_soft_bias_estimation"] = settings_->observer->doMagSoftBiasEstimation();
  params["do_baro_alt_bias_estimation"] = settings_->observer->doBaroAltBiasEstimation();
  params["do_gravity_estimation"] = settings_->observer->doGravityEstimation();
  params["imu_offset"] = Eigen::Vector3d::Zero().eval();   // TODO
  params["gnss_offset"] = Eigen::Vector3d::Zero().eval();  // TODO

  const auto config_dir = proj_paths_.cfgConfigDirPath();

  // For component
  const auto node_component = params;
  saveYamlNode(QDir(config_dir).filePath("observer_static.yaml"), node_component);

  // For standalone
  YAML::Node node_standalone(YAML::NodeType::Map);
  node_standalone["/**"][node::kObserver][kRosParamsKey] = params;
  saveYamlNode(QDir(config_dir).filePath("observer_static_standalone.yaml"), node_standalone);
}

void ProjectGenerator::generateControllerStaticConfig()
{
  const auto params = settings_->controller->staticParams();
  TOBAS_CHECK(params.IsMap());

  const auto config_dir = proj_paths_.cfgConfigDirPath();

  // For component
  const auto node_component = params;
  saveYamlNode(QDir(config_dir).filePath("controller_static.yaml"), node_component);

  // For standalone
  YAML::Node node_standalone(YAML::NodeType::Map);
  node_standalone["/**"][node::kController][kRosParamsKey] = params;
  saveYamlNode(QDir(config_dir).filePath("controller_static_standalone.yaml"), node_standalone);
}

void ProjectGenerator::generateMissionExecutorStaticConfig()
{
  const auto params = settings_->mission->staticParams();
  TOBAS_CHECK(params.IsMap());

  const auto config_dir = proj_paths_.cfgConfigDirPath();

  // For component
  const auto node_component = params;
  saveYamlNode(QDir(config_dir).filePath("mission_executor_static.yaml"), node_component);

  // For standalone
  YAML::Node node_standalone(YAML::NodeType::Map);
  node_standalone["/**"][node::kMissionExecutor][kRosParamsKey] = params;
  saveYamlNode(QDir(config_dir).filePath("mission_executor_static_standalone.yaml"), node_standalone);
}

void ProjectGenerator::generateRcTeleopStaticConfig()
{
  YAML::Node params(YAML::NodeType::Map);
  params["acrobat_mode"] = settings_->controller->acrobatModeCommand();
  params["stabilize_mode"] = settings_->controller->stabilizeModeCommand();
  params["loiter_mode"] = settings_->controller->loiterModeCommand();
  params["arm_duration"] = yaml::format(settings_->rc_input->armDuration());
  params["disarm_duration"] = yaml::format(settings_->rc_input->disarmDuration());

  const auto config_dir = proj_paths_.cfgConfigDirPath();

  // For component
  const auto node_component = params;
  saveYamlNode(QDir(config_dir).filePath("rc_teleop_static.yaml"), node_component);

  // For standalone
  YAML::Node node_standalone(YAML::NodeType::Map);
  node_standalone["/**"][node::kRcTeleop][kRosParamsKey] = params;
  saveYamlNode(QDir(config_dir).filePath("rc_teleop_static_standalone.yaml"), node_standalone);
}

void ProjectGenerator::generateImuFilterConfig()
{
  YAML::Node params(YAML::NodeType::Map);
  params["has_rpm_filter"] = settings_->hardware->hasRpmFilter();

  const auto config_dir = proj_paths_.cfgConfigDirPath();

  // For component
  const auto node_component = params;
  saveYamlNode(QDir(config_dir).filePath("imu_filter_static.yaml"), node_component);

  // For standalone
  YAML::Node node_standalone(YAML::NodeType::Map);
  node_standalone["/**"][node::kRcTeleop][kRosParamsKey] = params;
  saveYamlNode(QDir(config_dir).filePath("imu_filter_static_standalone.yaml"), node_standalone);
}

void ProjectGenerator::generateNetworkConfig()
{
  cmn::NetworkConfig config;
  config.interface = settings_->network->networkInterface();

  TOBAS_CHECK(config.save(proj_paths_.networkConfigPath()));
}

void ProjectGenerator::generateOriginalUadf()
{
  // Export the original UADF.
  const auto doc = uadf::exportUADF(uadf_);
  const auto robot = doc->RootElement();

  // Modify.
  replaceOriginalUadfMeshFilePaths(robot);

  // Save.
  TOBAS_CHECK(doc->SaveFile(proj_paths_.originalUadfPath().toUtf8().constData()) == tinyxml2::XML_SUCCESS);
}

void ProjectGenerator::generateModifiedUrdf()
{
  // Export the original URDF.
  const auto doc = urdf::exportUrdf(*uadf_.urdf);
  const auto robot = doc->RootElement();

  // Modify.
  resolveModifiedUrdfMeshFilePaths(robot);
  removePropellerJointLimits(robot);
  addXmlElements(robot);

  // Save.
  TOBAS_CHECK(doc->SaveFile(proj_paths_.xacroPath().toUtf8().constData()) == tinyxml2::XML_SUCCESS);
}

void ProjectGenerator::createEmptyFile(const QString& file_path)
{
  TOBAS_CHECK(path::createFilePath(file_path.toStdString(), true));
}

void ProjectGenerator::createEmptyYaml(const QString& file_path, bool overwrite)
{
  if (overwrite || !QFileInfo(file_path).isFile()) {
    saveYamlNode(file_path, YAML::Node(YAML::NodeType::Map));
  }
}

void ProjectGenerator::saveYamlNode(const QString& path, const YAML::Node& node)
{
  TOBAS_CHECK(yaml::save(path.toStdString(), node));
}

void ProjectGenerator::resolveModifiedUrdfMeshFilePaths(tinyxml2::XMLElement* elem)
{
  // Check the current element.
  resolveModifiedUrdfMeshFilePath(elem);

  // Check the child elements recursively.
  for (auto child = elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
    resolveModifiedUrdfMeshFilePaths(child);
  }
}

void ProjectGenerator::resolveModifiedUrdfMeshFilePath(tinyxml2::XMLElement* elem)
{
  if (std::strcmp(elem->Name(), "mesh") != 0) {
    return;
  }

  const auto filename = elem->Attribute("filename");
  if (!filename) {
    qWarning() << "Mesh element does not have attribute: \"filename\"";
    return;
  }

  const auto src_path = QString::fromStdString(urdf::resolveUri(filename).string());
  const QFileInfo src_info(src_path);
  if (!src_info.exists()) {
    qWarning() << "Mesh file" << src_path << "does not exist.";
    return;
  }

  // Copy mesh files under the Tobas package as needed.
  const auto base_name = src_info.fileName();
  const auto dst_path = QDir(proj_paths_.cfgMeshDirPath()).filePath(base_name);
  const QFileInfo dst_info(dst_path);
  if (dst_info.exists()) {
    // If `dst_path` exists but differs from `src_path`, delete it before copying because `QFile::copy` does not overwrite it.
    if (src_info.canonicalFilePath() != dst_info.canonicalFilePath()) {
      TOBAS_CHECK(QFile::remove(dst_path));
      TOBAS_CHECK(QFile::copy(src_path, dst_path));
    }
  }
  else {
    // If `dst_path` does not exist, simply copy it.
    TOBAS_CHECK(QFile::copy(src_path, dst_path));
  }

  // Replace mesh file paths.
  // Ignition cannot find paths in the `package://<pkg_name>` format,
  // so embed a xacro command to replace them with absolute paths.
  // cf. https://github.com/moveit/moveit_resources/blob/ros2/panda_description/urdf/panda.urdf.xacro
  const auto new_filename = "file://$(find " + proj_paths_.cfgPkgName() + ")/meshes/" + base_name;
  elem->SetAttribute("filename", new_filename.toUtf8().constData());
}

void ProjectGenerator::replaceOriginalUadfMeshFilePaths(tinyxml2::XMLElement* elem)
{
  // Check the current element.
  if (std::strcmp(elem->Name(), "mesh") != 0) {
    return;
  }

  // Check the child elements recursively.
  for (auto child = elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
    replaceOriginalUadfMeshFilePaths(child);
  }
}

void ProjectGenerator::replaceOriginalUadfMeshFilePath(tinyxml2::XMLElement* elem)
{
  const auto filename = elem->Attribute("filename");
  if (!filename) {
    qWarning() << "Mesh element does not have attribute: \"filename\"";
    return;
  }

  const auto src_path = QString::fromStdString(urdf::resolveUri(filename).string());
  const auto base_name = QFileInfo(src_path).fileName();

  // Specify as a relative path from `config_pkg`.
  const auto new_filename = "package://" + proj_paths_.cfgPkgName() + "/meshes/" + base_name;
  elem->SetAttribute("filename", new_filename.toUtf8().constData());
}

void ProjectGenerator::removePropellerJointLimits(tinyxml2::XMLElement* robot)
{
  std::set<std::string> prop_jnt_names;
  const auto& prop = settings_->propulsion_system;
  for (int i = 0; i < prop->numUnits(); ++i) {
    const auto link_name = prop->linkName(i).toStdString();
    const auto jnt_name = tree_.getSegment(link_name)->second.segment.joint().name;
    prop_jnt_names.insert(jnt_name);
  }

  for (auto child = robot->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (std::strcmp(child->Name(), "joint") == 0) {
      const auto jnt_name = child->Attribute("name");
      if (!jnt_name) {
        qWarning() << "Joint element does not have attribute: \"name\"";
        continue;
      }
      if (prop_jnt_names.contains(jnt_name)) {
        for (auto gchild = child->FirstChildElement(); gchild; gchild = gchild->NextSiblingElement()) {
          if (std::strcmp(gchild->Name(), "limit") == 0) {
            child->DeleteChild(gchild);
            break;
          }
        }
      }
    }
  }
}

void ProjectGenerator::addXmlElements(tinyxml2::XMLElement* robot)
{
  const auto ns = uadf_.urdf->getName() + "/$(arg ID)";
  const auto& root_name = tree_.getRootName();

  const auto& prop = settings_->propulsion_system;
  const auto& fmu = settings_->hardware;

  const auto drone = createDrone();

  // Get rotor link names.
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
    kImuSamplingRate,
    Eigen::Vector3d::Zero(),  // TODO: Offset
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
    Eigen::Vector3d::Zero(),  // TODO: Offset
    fmu->magNoiseStddev(),
    fmu->magHardBiasNorm());

  // Barometer plugin
  xml::addBarometerPlugin(
    robot,
    ns,
    root_name,
    fmu->presUpdateRate(),
    Eigen::Vector3d::Zero(),  // TODO: Offset
    fmu->presNoiseStddev());

  // GNSS plugin
  xml::addGnssPlugin(
    robot,
    ns,
    root_name,
    fmu->gnssUpdateRate(),
    Eigen::Vector3d::Zero(),  // TODO: Offset
    0.1,                      // The time for radio waves from GNSS satellites to reach the ground is roughly fixed.
    30.0,                     // TODO: What is the actual GNSS position correlation time constant?
    fmu->gnssHorizontalPositionAccuracy(),
    fmu->gnssVerticalPositionAccuracy(),
    fmu->gnssHorizontalVelocityStddev(),
    fmu->gnssVerticalVelocityStddev(),
    0.0  // TODO: Geoid Undulation
  );

  // Propulsion system plugins
  switch (drone.prop->type()) {
    case PropulsionSystem::kElectric: {
      const auto eprop = qt::qConstPointerCast<propulsion::electric::PropulsionSystemWidget>(prop->selected());
      const auto battery = eprop->battery;
      const auto units = eprop->units;

      // Battery plugin
      constexpr double kBatterySamplingRate = 100.0;  // TODO: Set the sampling rate in the GUI.
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
    case PropulsionSystem::kIce: {
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
  xml::addJointStateBroadcasterPlugin(
    robot, ns, joint_names, 100);  // TODO: Allow the update rate to be adjusted in the GUI.

  // Joint controller plugins
  for (const auto& [_, joint] : drone.joints) {
    if (!joint.isServoJoint()) {
      continue;
    }

    const auto limits = uadf_.urdf->getJoint(joint.name)->limits;
    if (!limits) {
      qWarning().nospace() << "No limit is set for the servo joint " + QString::fromStdString(joint.name) + ".";
      continue;
    }

    switch (joint.cmd_iface) {
      case JointCommandInterface::kNone: {
        break;
      }
      case JointCommandInterface::kPosition: {
        const auto max_vel = limits->velocity;
        TOBAS_CHECK(max_vel > 0.0);

        // Use the time required to rotate 60 deg at maximum speed as the time constant.
        // In other words, maximum speed is reached at 60 deg error.
        // TODO: Reproduce servo motor specifications such as no-load speed more accurately.
        const auto time_const = M_PI_3 / max_vel;
        xml::addJointPositionControllerPlugin(robot, ns, joint.name, joint.home_pos, time_const);
        break;
      }
      case JointCommandInterface::kVelocity: {
        xml::addJointVelocityControllerPlugin(robot, ns, joint.name, joint.home_pos);
        break;
      }
      case JointCommandInterface::kEffort: {
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

  // Base  joint for debug
  xml::addBaseStaticJoint(robot, tree_.getRootName());
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
