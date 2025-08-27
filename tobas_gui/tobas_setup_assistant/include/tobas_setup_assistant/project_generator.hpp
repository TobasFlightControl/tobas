#pragma once

#include <tinyxml2.h>

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp>
#include <tobas_drone_core/propulsion_system/ice_propulsion_system/ice_propulsion_system.hpp>
#include <tobas_kdl/tree.hpp>

#include "./settings.hpp"
#include "./template_generator.hpp"

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
    SettingsWidget* settings);

  bool generateProject(const std::filesystem::path& proj_path);

private:
  const rclcpp::Node::SharedPtr node_;
  const uadf::Model& uadf_;
  const kdl::Tree& tree_;
  SettingsWidget* const settings_;

  std::shared_ptr<TemplateGenerator> meta_env_;
  std::shared_ptr<TemplateGenerator> config_env_;
  std::shared_ptr<TemplateGenerator> user_msg_env_;
  std::shared_ptr<TemplateGenerator> user_cpp_env_;
  std::shared_ptr<TemplateGenerator> user_py_env_;

  /* ROS Packageのタブで指定されたTobasパッケージのパスへのエイリアス． */
  std::string flightActionsPackage() const;

  inja::json createTemplateData(const std::filesystem::path& proj_path);

  tobas::Drone createDrone();

  bool hasServoJoint() const;

  bool generateMetaPackage(const std::filesystem::path& proj_path, const inja::json& data);
  bool generateConfigPackage(const std::filesystem::path& proj_path, const inja::json& data);
  bool generateUserMsgPackage(const std::filesystem::path& proj_path, const inja::json& data);
  bool generateUserCppPackage(const std::filesystem::path& proj_path, const inja::json& data);
  bool generateUserPyPackage(const std::filesystem::path& proj_path, const inja::json& data);
  bool generateBackupFiles(const std::filesystem::path& proj_path);

  bool generateControllerManagerLaunch(const std::filesystem::path& proj_path);
  bool generateJointControllerManagerConfig(const std::filesystem::path& proj_path);
  bool generateJointControllerConfigs(const std::filesystem::path& proj_path);
  bool generateDroneConfig(const std::filesystem::path& proj_path);
  bool generatePreArmCheckConfig(const std::filesystem::path& proj_path);
  bool generateObserverStaticConfig(const std::filesystem::path& proj_path);
  bool generateControllerStaticConfig(const std::filesystem::path& proj_path);
  bool generateRcTeleopStaticConfig(const std::filesystem::path& proj_path);
  bool generateSshEndpointConfig(const std::filesystem::path& proj_path);
  bool generateOriginalUadf(const std::filesystem::path& proj_path);
  bool generateModifiedUrdf(const std::filesystem::path& proj_path);

  /* 空のファイルを作成する． */
  bool createEmptyFile(const std::filesystem::path& file_path);

  /* Map型で要素を持たないyamlファイルを作成する． */
  bool createEmptyYaml(const std::filesystem::path& file_path, bool overwrite = false);

  /* YAML::Nodeを保存する． */
  bool saveYamlNode(const std::filesystem::path& path, const YAML::Node& node);

  /* 全てのメッシュファイルのパスをパッケージ以下に変更する． */
  bool resolveModifiedUrdfMeshFilePaths(tinyxml2::XMLElement* elem, const std::filesystem::path& proj_path);

  /* オリジナルURDFの全てのメッシュファイルのパスをパッケージ以下に変更する． */
  bool replaceOriginalUadfMeshFilePaths(tinyxml2::XMLElement* elem, const std::filesystem::path& proj_path);

  /* プロペラジョイントのlimitタグを削除する． */
  bool removePropellerJointLimits(tinyxml2::XMLElement* robot);

  /* Gazeboプラグイン等をXMLに追加する． */
  bool addXmlElements(tinyxml2::XMLElement* robot, const std::filesystem::path& proj_path);

  void
  addJointControllerNode(tinyxml2::XMLElement* launch, const std::string& cfg_pkg_name, const std::string& ctrl_name);

  bool generateJointControllerConfig(
    const std::filesystem::path& proj_path,
    const std::string& jnt_name,
    const tobas::JointCommandInterface& cmd_iface);

  static std::string jointControllerName(const std::string& jnt_name);

  static tobas::TurningDirection turningDirectionUadfToTbsdrn(const uadf::Thrust::Direction& src);
};
}  // namespace sa
}  // namespace gui
