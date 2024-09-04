#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_path_tools/core.hpp>
#include <tobas_tools/package.hpp>
#include <tobas_tools/command.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>

#include "tobas_setup_assistant/package_generator.hpp"

#define DO_NOT_EDIT_THIS_PACKAGE "DO_NOT_EDIT_THIS_PACKAGE"

using namespace std;
namespace fs = filesystem;

namespace gui
{
namespace setup_assistant
{
PackageGenerator::PackageGenerator(const RobotInfo& robot, SettingsWidget* settings)
  : robot_(robot), settings_(settings)
{
  const auto pkg_path = fs::path(ament_index_cpp::get_package_share_directory(kPackageName));
  const auto templates_path = pkg_path / "templates";
  meta_env_ = make_shared<TemplateGenerator>(templates_path / "meta_package");
  cfg_env_ = make_shared<TemplateGenerator>(templates_path / "config_package");
  user_env_ = make_shared<TemplateGenerator>(templates_path / "user_package");
}

void PackageGenerator::generate()
{
  qt::ProgressDialog progress("Generate Package", 3, settings_);
  progress.setCancelButton(nullptr);
  progress.show();

  // Verify user settings
  progress.setLabelText("Verifying the validity of the user settings.");
  if (settings_->isValid())
  {
    progress.close();
    return;
  }
  progress.progressStep();

  // Generate Tobas package
  progress.setLabelText("Generating Tobas package.");
  generatePackage();
  progress.progressStep();

  // Build Tobas package
  progress.setLabelText("Building Tobas package.");
  if (!tobas::buildTobasPackage(tbsPath()))
  {
    progress.close();
    qt::qErrorBox(settings_, "Failed to build Tobas package.");
    return;
  }
  progress.progressStep();

  progress.close();
  qt::qInfoBox(settings_, "Tobas configuration package is generated and built successfully.");
}

string PackageGenerator::tbsPath() const
{
  return settings_->ros_package->tbsPath().toStdString();
}

inja::json PackageGenerator::createTemplateData()
{
  inja::json data;

  data["drone_name"] = robot_.robotName();

  // Controller
  data["controller_pkg"] = settings_->controller->controllerPackage();
  data["actions_pkg"] = settings_->controller->actionsPackage();

  // Observer
  data["observer_pkg"] = settings_->observer->observerPackage();

  // Hardware
  data["hardware_pkg"] = settings_->hardware->hardwarePackage();

  // Author Info
  data["author_name"] = settings_->author_info->authorName().toStdString();
  data["author_email"] = settings_->author_info->authorEmail().toStdString();

  // Ros Package
  data["meta_pkg_name"] = tobas::getTBSMetaName(tbsPath());
  data["config_pkg_name"] = tobas::getTBSConfigName(tbsPath());
  data["user_pkg_name"] = tobas::getTBSUserName(tbsPath());

  // Joint Controllers
  string joint_controllers = "joint_state_controller";
  for (int i = 0; i < settings_->custom_joints->count(); ++i)
  {
    const auto jnt_name = settings_->custom_joints->getJointName(i);
    joint_controllers += " " + jnt_name.toStdString() + "_controller";
  }
  data["joint_controllers"] = joint_controllers;

  return data;
}

void PackageGenerator::generatePackage()
{
  // Tobasパッケージを作成
  fs::create_directories(tbsPath());

  // テンプレート用アイテムを作成
  const auto data = createTemplateData();

  // メタパッケージを作成
  generateMetaPackage(data);

  // 設定パッケージを作成
  generateConfigPackage(data);

  // ユーザパッケージを作成
  generateUserPackage(data);
}

void PackageGenerator::generateMetaPackage(const inja::json& data)
{
  const auto meta_pkg_path = tobas::getTBSMetaPath(tbsPath());
  fs::create_directory(meta_pkg_path);

  meta_env_->generate(data, "CMakeLists.txt.tpl", meta_pkg_path);
  meta_env_->generate(data, "package.xml.tpl", meta_pkg_path);

  path::createFilePath(meta_pkg_path / DO_NOT_EDIT_THIS_PACKAGE);
}

void PackageGenerator::generateConfigPackage(const inja::json& data)
{
  // TODO
}

void PackageGenerator::generateUserPackage(const inja::json& data)
{
  // TODO
}

void PackageGenerator::generateDroneConfig(const fs::path& config_dir)
{
  // TODO
}

void PackageGenerator::generateJointControlConfig(const fs::path& config_dir)
{
  // TODO
}

void PackageGenerator::generateRCTeleopConfig(const fs::path& config_dir)
{
  // TODO
}

void PackageGenerator::generateControllerConfig(const fs::path& config_dir)
{
  // TODO
}

void PackageGenerator::generateObserverConfig(const fs::path& config_dir)
{
  // TODO
}

void PackageGenerator::generateURDFs(const fs::path& mesh_dir)
{
  // TODO
}

void PackageGenerator::resolveMeshFiles(tinyxml2::XMLElement* robot, const fs::path& mesh_dir)
{
  // TODO
}

void PackageGenerator::screenXMLElements(tinyxml2::XMLElement* robot)
{
  // TODO
}

bool PackageGenerator::isDeletableGazeboNode(tinyxml2::XMLElement* gazebo)
{
  // TODO
}

bool PackageGenerator::isDeletableGazeboPlugin(tinyxml2::XMLElement* plugin)
{
  // TODO
}

bool PackageGenerator::askRemoveOrKeepGazeboChild(tinyxml2::XMLElement* child)
{
  // TODO
}

void PackageGenerator::addXMLElements(tinyxml2::XMLElement* robot)
{
  // TODO
}
}  // namespace setup_assistant
}  // namespace gui
