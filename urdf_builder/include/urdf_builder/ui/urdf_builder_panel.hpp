#pragma once

#include <boost/filesystem.hpp>
#include <QtCore/QtCore>
#include <QtWidgets/QtWidgets>
#include <rviz/panel.h>

#include <tobas_property_tools/property_client.hpp>

#ifndef Q_MOC_RUN
#include <urdf/model.h>
#include "../ogre_helpers/static_link_updater.hpp"
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

  static constexpr char kConfigKey_LastOpenedDir[] = "last_opened_dir";

public:
  explicit URDFBuilderPanel(QWidget* parent = nullptr);
  ~URDFBuilderPanel() override;

  void onInitialize() override;
  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

  QStringList linkNames() const;
  QStringList jointNames() const;

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
  Ui::URDFBuilderPanelUI* ui_;
  view_model::URDFViewModel vm_;
  ogre_helpers::OgreControllerPtr ogre_ctrl_;
  QTimer* update_timer_;

  UpdateLinkDialog* link_dialog_;
  view_model::LinkViewModelPtr old_link_vm_;

  rclcpp::NodeHandle node_;
  ptree::PropertyClient property_client_;

  std::string getLastOpenedDir();
  void setLastOpenedDir(const std::string& file_path);

  void defineConnections();

  void reload();
  void reloadLinkTree();
  void reloadRobot();

  void addRootLink();
  bool saveURDF(const QString& file_path);

  bool isValid();
  bool isRobotNameValid();
  bool isJointsValid();

  static void collectUncheckedLinks(QTreeWidgetItem* item, std::unordered_set<std::string>& set);
};
}  // namespace ui
}  // namespace urdf_builder
