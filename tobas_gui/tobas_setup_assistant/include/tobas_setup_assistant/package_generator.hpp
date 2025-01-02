#pragma once

#include <tinyxml2.h>

#include <tobas_drone_core/drone.hpp>

#include "./robot_info.hpp"
#include "./settings.hpp"
#include "./template_generator.hpp"

namespace gui
{
namespace setup_assistant
{
class PackageGenerator
{
  static constexpr char kROSParamsKey[] = "ros__parameters";

  static constexpr char kDoNotEditThisPackage[] = "DO_NOT_EDIT_THIS_PACKAGE";
  static constexpr char kYouCanEditThisPackage[] = "YOU_CAN_EDIT_THIS_PACKAGE";

public:
  explicit PackageGenerator(rclcpp::Node::SharedPtr node, RobotInfo& robot, SettingsWidget* settings);

  bool generatePackage();

private:
  const rclcpp::Node::SharedPtr node_;
  RobotInfo& robot_;
  SettingsWidget* settings_;

  std::shared_ptr<TemplateGenerator> meta_env_;
  std::shared_ptr<TemplateGenerator> config_env_;
  std::shared_ptr<TemplateGenerator> user_cpp_env_;
  std::shared_ptr<TemplateGenerator> user_py_env_;

  /* ROS Packageのタブで指定されたTobasパッケージのパスへのエイリアス． */
  std::string tbsPath() const;
  std::string flightActionsPackage() const;

  inja::json createTemplateData();
  tobas::Drone createDrone();

  bool hasServoJoint() const;

  bool generateBackupFiles();
  bool generateMetaPackage(const inja::json& data);
  bool generateConfigPackage(const inja::json& data);
  bool generateUserCppPackage(const inja::json& data);
  bool generateUserPyPackage(const inja::json& data);

  bool generateControllerManagerLaunch(const std::filesystem::path& launch_dir);
  bool generateGazeboJointCommandHandlerConfig(const std::filesystem::path& config_dir);
  bool generateJointControllerManagerConfig(const std::filesystem::path& config_dir);
  bool generateJointControllerConfigs(const std::filesystem::path& config_dir);
  bool generateDroneConfig(const std::filesystem::path& config_dir);
  bool generateRCTeleopConfig(const std::filesystem::path& config_dir);
  bool generateControllerStaticConfig(const std::filesystem::path& config_dir);
  bool generateObserverStaticConfig(const std::filesystem::path& config_dir);
  bool generateURDF(const std::filesystem::path& mesh_dir);

  /* 空のファイルを作成する． */
  bool createEmptyFile(const std::filesystem::path& file_path);

  /* Map型で要素を持たないyamlファイルを作成する． */
  bool createEmptyYaml(const std::filesystem::path& file_path, bool overwrite = false);

  /* YAML::Nodeを保存する． */
  bool saveYamlNode(const std::filesystem::path& path, const YAML::Node& node);

  /* プロペラジョイントのlimitタグを削除する． */
  bool removePropellerJointLimits(tinyxml2::XMLElement* robot);

  /* 全てのメッシュファイルのパスをパッケージ以下に変更する． */
  bool resolveMeshFiles(tinyxml2::XMLElement* elem, const std::filesystem::path& mesh_dir);

  /* Gazeboプラグイン等をXMLに追加する． */
  bool addXMLElements(tinyxml2::XMLElement* robot);
};
}  // namespace setup_assistant
}  // namespace gui
