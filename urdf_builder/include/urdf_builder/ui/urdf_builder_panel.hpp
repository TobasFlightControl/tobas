#pragma once

#include <boost/filesystem.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <QListWidgetItem>
#include <QTreeWidgetItem>
#include <QTimer>
#include <rviz/panel.h>

#ifndef Q_MOC_RUN
#include <urdf/model.h>
#include "../rviz_helpers/static_link_updater.hpp"
#include "../view_model/urdf_view_model.hpp"
#include "../ogre_helpers/ogre_controller.hpp"
#include "./update_link_dialog.hpp"
#endif

namespace Ui
{
class URDFBuilderPanelUI;
}

namespace urdf_builder
{
namespace ui
{
class LinkTreeWidgetItem;

class URDFBuilderPanel : public rviz::Panel
{
  Q_OBJECT

  static constexpr char kConfigPath[] = "~/.config/urdf_builder/config.ini";
  static constexpr char kConfigKey_LastOpenedDir[] = "last_opened_dir";

public:
  explicit URDFBuilderPanel(QWidget* parent = nullptr);

  ~URDFBuilderPanel() override;

  void onInitialize() override;
  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

private Q_SLOTS:
  void RobotNameTextChanged(const QString& name);
  void NewButtonClicked();
  void LoadButtonClicked();
  void SaveButtonClicked();
  void SaveAsButtonClicked();
  void EnableVisualCheckBoxToggled(bool checked);
  void EnableCollisionCheckBoxToggled(bool checked);
  void LinkTreeWidgetItemClicked(QTreeWidgetItem* item, int column);
  void LinkTreeWidgetItemChanged(QTreeWidgetItem* item, int column);
  void LinkTreeContextMenuRequested(const QPoint&);
  void AddLinkActionToggled(bool);
  void RemoveLinkActionToggled(bool);
  void CloneLinkActionToggled(bool);
  void OnUpdate();
  void LinkDialogChanged();

private:
  const std::string config_path_;
  boost::property_tree::ptree pt_;

  Ui::URDFBuilderPanelUI* ui_;
  view_model::URDFViewModel vm_;
  ogre_helpers::OgreControllerPtr ogre_ctrl_;
  QTimer* update_timer_;

  UpdateLinkDialog* link_dialog_;
  view_model::LinkViewModelPtr old_link_vm_;

  void createConfig();
  std::string getLastOpenedDir();
  void setLastOpenedDir(const std::string& file_path);

  void defineConnections();

  void reload();
  void reloadLinkTree();
  void reloadRobot();

  void addRootLink();
  bool saveURDF(const QString& file_path);

  bool isValid();
  bool isRobotNameVolid();
  bool isJointsValid();

  static void collectUncheckedLinks(QTreeWidgetItem* item, std::unordered_set<std::string>& set);
};
}  // namespace ui
}  // namespace urdf_builder
