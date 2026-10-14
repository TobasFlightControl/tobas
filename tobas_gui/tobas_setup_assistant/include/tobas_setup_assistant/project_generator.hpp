// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>

#include <tinyxml2.h>
#include <QString>

#include <tobas_drone_core/drone.hpp>
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
    const uadf::Model& _uadf,
    const kdl::Tree& _tree,
    const SettingsWidget* _settings,
    QWidget* _parent);

  void generateProject(const QString& _proj_path);

  void setClearDynamicParams(bool _flag);

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

  void generateMetaPackage(const inja::json& _data);
  void generateConfigPackage(const inja::json& _data);
  void generateUserMsgPackage(const inja::json& _data);
  void generateUserCppPackage(const inja::json& _data);
  void generateUserPyPackage(const inja::json& _data);
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

  /** Create an empty file. */
  void createEmptyFile(const QString& _file_path);

  /** Create a YAML file with a map type and no elements. */
  void createEmptyYaml(const QString& _file_path, bool _overwrite);

  /** Save `YAML::Node`. */
  void saveYamlNode(const QString& _path, const YAML::Node& _node);

  /** Change mesh file paths to paths under the package. */
  void resolveModifiedUrdfMeshFilePaths(tinyxml2::XMLElement* _elem);
  void resolveModifiedUrdfMeshFilePath(tinyxml2::XMLElement* _elem);

  /** Change mesh file paths in the original URDF to paths under the package. */
  void replaceOriginalUadfMeshFilePaths(tinyxml2::XMLElement* _elem);
  void replaceOriginalUadfMeshFilePath(tinyxml2::XMLElement* _elem);

  /** Remove `limit` tags from propeller joints. */
  void removePropellerJointLimits(tinyxml2::XMLElement* _robot);

  /** Add Gazebo plugins and related elements to XML. */
  void addXmlElements(tinyxml2::XMLElement* _robot);
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
