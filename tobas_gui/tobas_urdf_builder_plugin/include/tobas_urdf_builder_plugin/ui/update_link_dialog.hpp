// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>

#include <QDialog>
#include <QListWidgetItem>
#include <QSettings>

#include "../view_model/joint_view_model.hpp"
#include "../view_model/link_view_model.hpp"

namespace Ui
{
class UpdateLinkDialogUI;
using UpdateLinkDialogUIPtr = std::shared_ptr<UpdateLinkDialogUI>;
}  // namespace Ui

namespace tobas::gui::ub::ui
{
class UrdfBuilderPanel;

class UpdateLinkDialog : public QDialog
{
  Q_OBJECT

  using self = UpdateLinkDialog;
  using super = QDialog;

Q_SIGNALS:
  void Changed();

public:
  explicit UpdateLinkDialog(UrdfBuilderPanel* _main);

  void done(int _code) override;

  void readFromVM(const view_model::LinkViewModelPtr& _link_vm);
  void setTabsEnabled(bool _enabled);

  const view_model::LinkViewModelPtr& viewModel() const;

private Q_SLOTS:
  void onVisualGeometryTypeComboBoxIndexChanged(int _index);
  void onCollisionGeometryTypeComboBoxIndexChanged(int _index);
  void onJointParentComboBoxIndexChanged(int _index);
  void onJointTypeComboBoxIndexChanged(int _index);
  void onJointSpinBoxValueChanged(double);
  void onVisualSpinBoxValueChanged(double);
  void onCollisionSpinBoxValueChanged(double);
  void onInertialSpinBoxValueChanged(double);
  void onLinkNameLineEditTextChanged(const QString& _text);
  void onJointNameLineEditTextChanged(const QString& _text);
  void onVisualNameLineEditTextChanged(const QString& _text);
  void onVisualGeometryMeshPathLineEditTextChanged(const QString& _text);
  void onCollisionNameLineEditTextChanged(const QString& _text);
  void onCollisionGeometryMeshPathEditTextChanged(const QString& _text);
  void onMaterialNameLineEditTextChanged(const QString& _text);
  void onMaterialTexturePathLineEditTextChanged(const QString& _text);
  void onVisualListWidgetItemClicked(QListWidgetItem*);
  void onCollisionListWidgetItemClicked(QListWidgetItem*);
  void onRenameLinkButtonClicked();
  void onRenameJointButtonClicked();
  void onAddVisualButtonClicked();
  void onRemoveVisualButtonClicked();
  void onAddCollisionButtonClicked();
  void onRemoveCollisionButtonClicked();
  void onVisualGeometryMeshBrowseButtonClicked();
  void onCollisionGeometryMeshBrowseButtonClicked();
  void onMaterialColorPickButtonClicked();
  void onBuildInertiaBoxButtonClicked();
  void onBuildInertiaCylinderButtonClicked();
  void onBuildInertiaSphereButtonClicked();

private:
  QSettings settings_store_;

  UrdfBuilderPanel* main_;
  Ui::UpdateLinkDialogUIPtr ui_;
  view_model::LinkViewModelPtr link_vm_;
  view_model::VisualViewModelPtr visual_vm_;
  view_model::CollisionViewModelPtr collision_vm_;

  struct
  {
    std::map<QString, QFrame*> visual_geom;
    std::map<QString, QFrame*> collision_geom;
  } frame_map_;

  void defineConnections();

  void readFromVM(const view_model::JointViewModelPtr& _joint);
  void readFromVM(const view_model::VisualViewModelPtr& _visual);
  void readFromVM(const view_model::CollisionViewModelPtr& _collision);
  void readFromVM(const view_model::InertialViewModelPtr& _inertial);

  void readFromUI(const view_model::JointViewModelPtr& _joint) const;
  void readFromUI(const view_model::VisualViewModelPtr& _visual) const;
  void readFromUI(const view_model::CollisionViewModelPtr& _collision) const;
  void readFromUI(const view_model::InertialViewModelPtr& _inertial) const;

  void blockSignals(bool _block);
  void emitChanged();

  static void arrangeVisualGeometryTypeFrames(const std::map<QString, QFrame*>& _map, const QString& _type);
};
}  // namespace tobas::gui::ub::ui
