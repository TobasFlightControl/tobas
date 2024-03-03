#pragma once

#include <map>
#include <QDialog>
#include <QFrame>
#include <QListWidgetItem>

#include "../view_model/link_view_model.hpp"
#include "../view_model/joint_view_model.hpp"

namespace Ui
{
class UpdateLinkDialogUI;

using UpdateLinkDialogUIPtr = std::shared_ptr<UpdateLinkDialogUI>;
}  // namespace Ui

namespace urdf_builder
{
namespace ui
{
class UpdateLinkDialog : public QDialog
{
  Q_OBJECT

Q_SIGNALS:
  void Changed();

public:
  explicit UpdateLinkDialog(const view_model::LinkViewModelPtr& vm, QWidget* parent = nullptr);

  void done(int) override;

  void readFromVM(const view_model::LinkViewModelPtr& vm);
  void setTabsEnabled(bool enabled);

  const view_model::LinkViewModelPtr& viewModel() const;

private Q_SLOTS:
  void VisualGeometryTypeComboBoxIndexChanged(int index);
  void CollisionGeometryTypeComboBoxIndexChanged(int index);
  void JointParentComboBoxIndexChanged(int index);
  void JointTypeComboBoxIndexChanged(int index);
  void JointSpinBoxValueChanged(double);
  void VisualSpinBoxValueChanged(double);
  void CollisionSpinBoxValueChanged(double);
  void InertialSpinBoxValueChanged(double);
  void LinkNameLineEditTextChanged(const QString& text);
  void JointNameLineEditTextChanged(const QString& text);
  void VisualNameLineEditTextChanged(const QString& text);
  void VisualGeometryMeshPathLineEditTextChanged(const QString& text);
  void CollisionNameLineEditTextChanged(const QString& text);
  void CollisionGeometryMeshPathEditTextChanged(const QString& text);
  void MaterialNameLineEditTextChanged(const QString& text);
  void MaterialTexturePathLineEditTextChanged(const QString& text);
  void VisualListWidgetItemClicked(QListWidgetItem*);
  void CollisionListWidgetItemClicked(QListWidgetItem*);
  void RenameLinkButtonClicked();
  void RenameJointButtonClicked();
  void AddVisualButtonClicked();
  void RemoveVisualButtonClicked();
  void AddCollisionButtonClicked();
  void RemoveCollisionButtonClicked();
  void VisualGeometryMeshBrowseButtonClicked();
  void CollisionGeometryMeshBrowseButtonClicked();
  void MaterialColorPickButtonClicked();
  void BuildInertiaBoxButtonClicked();
  void BuildInertiaCylinderButtonClicked();
  void BuildInertiaSphereButtonClicked();

private:
  Ui::UpdateLinkDialogUIPtr ui_;
  view_model::LinkViewModelPtr vm_;
  view_model::VisualViewModelPtr vvm_;
  view_model::CollisionViewModelPtr cvm_;

  struct
  {
    std::map<QString, QFrame*> visual_geom;
    std::map<QString, QFrame*> collision_geom;
  } frame_map_;

  void defineConnections();

  void readFromVM(const view_model::JointViewModelPtr& joint);
  void readFromVM(const view_model::VisualViewModelPtr& visual);
  void readFromVM(const view_model::CollisionViewModelPtr& collision);
  void readFromVM(const view_model::InertialViewModelPtr& inertial);

  void readFromUI(const view_model::JointViewModelPtr& joint);
  void readFromUI(const view_model::VisualViewModelPtr& visual);
  void readFromUI(const view_model::CollisionViewModelPtr& collision);
  void readFromUI(const view_model::InertialViewModelPtr& inertial);

  void blockSignals(bool block);
  void emitChanged();

  static void
  arrangeVisualGeometryTypeFrames(const std::map<QString, QFrame*>& map, const QString& typeName);
};
}  // namespace ui
}  // namespace urdf_builder
