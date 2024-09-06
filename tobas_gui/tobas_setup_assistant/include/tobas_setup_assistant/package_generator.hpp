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

  void generate();

private:
  rclcpp::Node::SharedPtr node_;
  RobotInfo& robot_;
  SettingsWidget* settings_;

  std::shared_ptr<TemplateGenerator> meta_env_;
  std::shared_ptr<TemplateGenerator> cfg_env_;
  std::shared_ptr<TemplateGenerator> user_env_;

  /* ROS Packageのタブで指定されたTobasパッケージのパスへのエイリアス． */
  std::string tbsPath() const;

  inja::json createTemplateData();
  tobas::Drone createDrone();

  bool generatePackage();
  bool generateMetaPackage(const inja::json& data);
  bool generateConfigPackage(const inja::json& data);
  bool generateUserPackage(const inja::json& data);

  bool generateDroneConfig(const std::filesystem::path& config_dir);
  bool generateJointControlConfig(const std::filesystem::path& config_dir);
  bool generateRCTeleopConfig(const std::filesystem::path& config_dir);
  bool generateControllerConfig(const std::filesystem::path& config_dir);
  bool generateObserverConfig(const std::filesystem::path& config_dir);
  bool generateURDFs(const std::filesystem::path& mesh_dir);

  /* 空のファイルを作成する． */
  bool createEmptyFile(const std::filesystem::path& file_path);

  /* YAML::Nodeを保存する． */
  bool saveYamlNode(const std::filesystem::path& path, const YAML::Node& node);

  /* 全てのメッシュファイルのパスをパッケージ以下に変更する． */
  bool resolveMeshFiles(tinyxml2::XMLElement* elem, const std::filesystem::path& mesh_dir);

  /* 悪影響を与えるかもしれないXML要素を，ユーザに確認した上で消す． */
  bool screenXMLElements(tinyxml2::XMLElement* robot);

  /* プラグインを強制削除すべきかどうかを判定する． */
  bool isDeletableGazeboPlugin(tinyxml2::XMLElement* plugin);

  /* Gazeboの子要素を削除してよいかユーザに尋ねる． */
  bool askRemoveOrKeepGazeboChild(tinyxml2::XMLElement* child);

  /* Gazeboプラグイン等をXMLに追加する． */
  bool addXMLElements(tinyxml2::XMLElement* robot);
};
}  // namespace setup_assistant
}  // namespace gui
