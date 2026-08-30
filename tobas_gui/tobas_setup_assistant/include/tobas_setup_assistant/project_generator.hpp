// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>

#include <tinyxml2.h>
#include <QString>

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp>
#include <tobas_drone_core/propulsion_system/ice_propulsion_system/ice_propulsion_system.hpp>
#include <tobas_gui_common/project_paths.hpp>
#include <tobas_kdl/tree.hpp>

#include "./settings.hpp"
#include "./template_generator.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class ProjectGenerator
{
  static constexpr char kRosParamsKey[] = "ros__parameters";

  static constexpr char kDoNotEditThisPackage[] = "DO_NOT_EDIT_THIS_PACKAGE";
  static constexpr char kYouCanEditThisPackage[] = "YOU_CAN_EDIT_THIS_PACKAGE";

public:
  explicit ProjectGenerator(
    const uadf::Model& uadf,
    const kdl::Tree& tree,
    const SettingsWidget* settings,
    QWidget* parent);

  bool generateProject(const QString& proj_path);

  void setClearDynamicParams(bool flag);

private:
  const uadf::Model& uadf_;
  const kdl::Tree& tree_;
  const SettingsWidget* const settings_;
  QWidget* const parent_;

  struct Config
  {
    bool clear_dynamic_params = false;
  } config_;

  cmn::ProjectPaths proj_paths_;

  std::optional<TemplateGenerator> meta_env_;
  std::optional<TemplateGenerator> config_env_;
  std::optional<TemplateGenerator> user_msg_env_;
  std::optional<TemplateGenerator> user_cpp_env_;
  std::optional<TemplateGenerator> user_py_env_;

  inja::json createTemplateData() const;
  Drone createDrone() const;
  bool hasServoJoint() const;

  bool generateMetaPackage(const inja::json& data);
  bool generateConfigPackage(const inja::json& data);
  bool generateUserMsgPackage(const inja::json& data);
  bool generateUserCppPackage(const inja::json& data);
  bool generateUserPyPackage(const inja::json& data);
  bool generateBackupFiles();

  bool generateDroneConfig();
  bool generateHealthMonitorConfig();
  bool generateRotorAnomalyDetectorConfig();
  bool generateObserverStaticConfig();
  bool generateControllerStaticConfig();
  bool generateMissionExecutorStaticConfig();
  bool generateRcTeleopStaticConfig();
  bool generateImuFilterConfig();
  bool generateNetworkConfig();
  bool generateOriginalUadf();
  bool generateModifiedUrdf();

  /* Create an empty file. */
  bool createEmptyFile(const QString& file_path);

  /* Create a YAML file with a map type and no elements. */
  bool createEmptyYaml(const QString& file_path, bool overwrite);

  /* Save `YAML::Node`. */
  bool saveYamlNode(const QString& path, const YAML::Node& node);

  /* Change all mesh file paths to paths under the package. */
  bool resolveModifiedUrdfMeshFilePaths(tinyxml2::XMLElement* elem);

  /* Change all mesh file paths in the original URDF to paths under the package. */
  bool replaceOriginalUadfMeshFilePaths(tinyxml2::XMLElement* elem);

  /* Remove `limit` tags from propeller joints. */
  bool removePropellerJointLimits(tinyxml2::XMLElement* robot);

  /* Add Gazebo plugins and related elements to XML. */
  bool addXmlElements(tinyxml2::XMLElement* robot);

  static TurningDirection turningDirectionUadfToTbsdrn(const uadf::Thrust::Direction& src);
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
