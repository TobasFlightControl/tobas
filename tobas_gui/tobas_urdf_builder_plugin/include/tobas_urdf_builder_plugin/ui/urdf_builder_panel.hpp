#pragma once

#include <rviz_common/panel.hpp>

#include <tobas_ros2_tools/async_node_manager.hpp>
#include <tobas_property_client/property_client.hpp>

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

namespace gui
{
namespace urdf_builder
{
namespace ui
{
/**
 * @brief Rvizのメインウィジェットにプラグインするメインパネル．
 */
class URDFBuilderPanel : public rviz_common::Panel
{
  Q_OBJECT

  using self = URDFBuilderPanel;
  using super = rviz_common::Panel;

  static constexpr char kConfigKey_LastOpenedDir[] = "last_opened_dir";

public:
  explicit URDFBuilderPanel(QWidget* parent = nullptr);
  ~URDFBuilderPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

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
  void EnableInertiaCheckBoxToggled(bool checked);
  void LinkTreeWidgetItemClicked(QTreeWidgetItem* item, int column);
  void LinkTreeWidgetItemChanged(QTreeWidgetItem* item, int column);
  void LinkTreeContextMenuRequested(const QPoint&);
  void AddLinkActionToggled(bool);
  void RemoveLinkActionToggled(bool);
  void CloneLinkActionToggled(bool);
  void OnUpdate();
  void LinkDialogChanged();

private:
  ros2::AsyncNodeManager node_manager_;  // Qtと別のスレッドで動作するノード
  const rclcpp::Node::SharedPtr node_;
  ptree::PropertyClient property_client_;

  Ui::URDFBuilderPanelUI* ui_;
  view_model::URDFViewModel vm_;
  ogre::OgreController::SharedPtr ogre_ctrl_;

  UpdateLinkDialog* link_dialog_;
  view_model::LinkViewModelPtr old_link_vm_;

  QTimer update_timer_;

  QString getLastOpenedDir();
  void setLastOpenedDir(const QString& file_path);

  void defineConnections();

  void reload();
  void reloadLinkTree();
  void reloadRobot();

  void addRootLink();
  void selectRootLink();
  void selectLink(QTreeWidgetItem* item);
  void reflectSelectedItem(QTreeWidgetItem* item);
  bool saveURDF(const QString& file_path);

  bool isValid();
  bool isRobotNameValid();
  bool isJointsValid();

  static void collectUncheckedLinks(QTreeWidgetItem* item, QSet<QString>& set);
};
}  // namespace ui
}  // namespace urdf_builder
}  // namespace gui
