// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>

#include <urdf/model.h>
#include <QSettings>
#include <QTimer>
#include <QTreeWidgetItem>
#include <rviz_common/panel.hpp>

#include "../ogre_helpers/ogre_controller.hpp"
#include "../ogre_helpers/static_link_updater.hpp"
#include "../view_model/urdf_view_model.hpp"
#include "./update_link_dialog.hpp"

namespace Ui
{
class URDFBuilderPanelUI;
}  // namespace Ui

namespace tobas
{
namespace gui
{
namespace ub
{
namespace ui
{
/**
 * @brief Main panel embedded in the RViz window.
 */
class UrdfBuilderPanel : public rviz_common::Panel
{
  Q_OBJECT

  using self = UrdfBuilderPanel;
  using super = rviz_common::Panel;

  static constexpr char kConfigKey_LastOpenedDir[] = "urdf_builder_panel/last_opened_dir";

public:
  explicit UrdfBuilderPanel(QWidget* parent = nullptr);
  ~UrdfBuilderPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

  QStringList linkNames() const;
  QStringList jointNames() const;

private Q_SLOTS:
  void onRobotNameTextChanged(const QString& name);
  void onNewButtonClicked();
  void onLoadButtonClicked();
  void onSaveButtonClicked();
  void onSaveAsButtonClicked();
  void onEnableVisualCheckBoxToggled(bool checked);
  void onEnableCollisionCheckBoxToggled(bool checked);
  void onEnableInertiaCheckBoxToggled(bool checked);
  void onLinkTreeWidgetItemClicked(QTreeWidgetItem* item, int column);
  void onLinkTreeWidgetItemChanged(QTreeWidgetItem* item, int column);
  void onLinkTreeContextMenuRequested(const QPoint&);
  void onAddLinkActionToggled(bool);
  void onRemoveLinkActionToggled(bool);
  void onCloneLinkActionToggled(bool);
  void onUpdate();
  void onLinkDialogChanged();

private:
  QSettings settings_store_;

  Ui::URDFBuilderPanelUI* ui_;
  view_model::URDFViewModel vm_;
  std::optional<ogre::OgreController> ogre_ctrl_;

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
}  // namespace ub
}  // namespace gui
}  // namespace tobas
