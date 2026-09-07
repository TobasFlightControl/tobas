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
public:
  explicit ProjectGenerator(
    const uadf::Model& uadf,
    const kdl::Tree& tree,
    const SettingsWidget* settings,
    QWidget* parent);

  void generateProject(const QString& proj_path);

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

  void generateMetaPackage(const inja::json& data);
  void generateConfigPackage(const inja::json& data);
  void generateUserMsgPackage(const inja::json& data);
  void generateUserCppPackage(const inja::json& data);
  void generateUserPyPackage(const inja::json& data);
  void generateBackupFiles();

  void generateDroneConfig();
  void generateHealthMonitorConfig();
  void generateRotorAnomalyDetectorConfig();
  void generateObserverStaticConfig();
  void generateControllerStaticConfig();
  void generateMissionExecutorStaticConfig();
  void generateRcTeleopStaticConfig();
  void generateImuFilterConfig();
  void generateNetworkConfig();
  void generateOriginalUadf();
  void generateModifiedUrdf();

  /* Create an empty file. */
  void createEmptyFile(const QString& file_path);

  /* Create a YAML file with a map type and no elements. */
  void createEmptyYaml(const QString& file_path, bool overwrite);

  /* Save `YAML::Node`. */
  void saveYamlNode(const QString& path, const YAML::Node& node);

  /* Change mesh file paths to paths under the package. */
  void resolveModifiedUrdfMeshFilePaths(tinyxml2::XMLElement* elem);
  void resolveModifiedUrdfMeshFilePath(tinyxml2::XMLElement* elem);

  /* Change mesh file paths in the original URDF to paths under the package. */
  void replaceOriginalUadfMeshFilePaths(tinyxml2::XMLElement* elem);
  void replaceOriginalUadfMeshFilePath(tinyxml2::XMLElement* elem);

  /* Remove `limit` tags from propeller joints. */
  void removePropellerJointLimits(tinyxml2::XMLElement* robot);

  /* Add Gazebo plugins and related elements to XML. */
  void addXmlElements(tinyxml2::XMLElement* robot);
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
