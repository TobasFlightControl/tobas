#pragma once

#include <tinyxml2.h>

#include "./robot_info.hpp"
#include "./settings.hpp"
#include "./template_generator.hpp"

namespace gui
{
namespace setup_assistant
{
class PackageGenerator
{
public:
  explicit PackageGenerator(const RobotInfo& robot, SettingsWidget* settings);

  void generate();

private:
  const RobotInfo& robot_;
  SettingsWidget* settings_;

  std::shared_ptr<TemplateGenerator> meta_env_;
  std::shared_ptr<TemplateGenerator> cfg_env_;
  std::shared_ptr<TemplateGenerator> user_env_;

  /* ROS Packageのタブで指定されたTobasパッケージのパスへのエイリアス． */
  std::string tbsPath() const;

  inja::json createTemplateData();

  void generatePackage();
  void generateMetaPackage(const inja::json& data);
  void generateConfigPackage(const inja::json& data);
  void generateUserPackage(const inja::json& data);

  void generateDroneConfig(const std::filesystem::path& config_dir);
  void generateJointControlConfig(const std::filesystem::path& config_dir);
  void generateRCTeleopConfig(const std::filesystem::path& config_dir);
  void generateControllerConfig(const std::filesystem::path& config_dir);
  void generateObserverConfig(const std::filesystem::path& config_dir);
  void generateURDFs(const std::filesystem::path& mesh_dir);

  /* 全てのメッシュファイルのパスをパッケージ以下に変更する． */
  void resolveMeshFiles(tinyxml2::XMLElement* robot, const std::filesystem::path& mesh_dir);

  /* 悪影響を与えるかもしれないXML要素を，ユーザに確認した上で消す． */
  void screenXMLElements(tinyxml2::XMLElement* robot);

  bool isDeletableGazeboNode(tinyxml2::XMLElement* gazebo);

  /* プラグインを強制削除すべきかどうかを判定する． */
  bool isDeletableGazeboPlugin(tinyxml2::XMLElement* plugin);

  /* 属性を確認した上でGazeboの子ノードを削除する． */
  bool askRemoveOrKeepGazeboChild(tinyxml2::XMLElement* child);

  /* Gazeboプラグイン等をXMLに追加する． */
  void addXMLElements(tinyxml2::XMLElement* robot);
};
}  // namespace setup_assistant
}  // namespace gui
