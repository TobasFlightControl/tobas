// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tinyxml2.h>

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
    rclcpp::Node::SharedPtr node,
    const uadf::Model& uadf,
    const kdl::Tree& tree,
    const SettingsWidget* settings,
    QWidget* parent);

  bool generateProject(const std::filesystem::path& proj_path);

  void setClearDynamicParams(bool flag);

private:
  const rclcpp::Node::SharedPtr node_;
  const uadf::Model& uadf_;
  const kdl::Tree& tree_;
  const SettingsWidget* const settings_;
  QWidget* const parent_;

  struct Config
  {
    bool clear_dynamic_params = false;
  } config_;

  cmn::ProjectPaths proj_paths_;

  std::shared_ptr<TemplateGenerator> meta_env_;
  std::shared_ptr<TemplateGenerator> config_env_;
  std::shared_ptr<TemplateGenerator> user_msg_env_;
  std::shared_ptr<TemplateGenerator> user_cpp_env_;
  std::shared_ptr<TemplateGenerator> user_py_env_;

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
  bool generateSshConfig();
  bool generateNetworkConfig();
  bool generateOriginalUadf();
  bool generateModifiedUrdf();

  /* 空のファイルを作成する． */
  bool createEmptyFile(const std::filesystem::path& file_path);

  /* Map型で要素を持たないyamlファイルを作成する． */
  bool createEmptyYaml(const std::filesystem::path& file_path, bool overwrite);

  /* YAML::Nodeを保存する． */
  bool saveYamlNode(const std::filesystem::path& path, const YAML::Node& node);

  /* 全てのメッシュファイルのパスをパッケージ以下に変更する． */
  bool resolveModifiedUrdfMeshFilePaths(tinyxml2::XMLElement* elem);

  /* オリジナルURDFの全てのメッシュファイルのパスをパッケージ以下に変更する． */
  bool replaceOriginalUadfMeshFilePaths(tinyxml2::XMLElement* elem);

  /* プロペラジョイントのlimitタグを削除する． */
  bool removePropellerJointLimits(tinyxml2::XMLElement* robot);

  /* Gazeboプラグイン等をXMLに追加する． */
  bool addXmlElements(tinyxml2::XMLElement* robot);

  static TurningDirection turningDirectionUadfToTbsdrn(const uadf::Thrust::Direction& src);
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
