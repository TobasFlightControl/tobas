#pragma once

#include <map>
#include <QtWidgets/QtWidgets>

#include <tobas_property_tools/property_client.hpp>

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
class URDFBuilderPanel;

class UpdateLinkDialog : public QDialog
{
  Q_OBJECT

  static constexpr char kConfigKey_VisualGeometryMeshBrowseDir[] = "visual_geometry_mesh_dir";
  static constexpr char kConfigKey_CollisionGeometryMeshBrowseDir[] = "collision_geometry_mesh_dir";

Q_SIGNALS:
  void Changed();

public:
  explicit UpdateLinkDialog(URDFBuilderPanel* main);

  void done(int code) override;

  void readFromVM(const view_model::LinkViewModelPtr& link_vm);
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
  URDFBuilderPanel* main_;
  Ui::UpdateLinkDialogUIPtr ui_;
  view_model::LinkViewModelPtr link_vm_;
  view_model::VisualViewModelPtr visual_vm_;
  view_model::CollisionViewModelPtr collision_vm_;

  struct
  {
    std::map<QString, QFrame*> visual_geom;
    std::map<QString, QFrame*> collision_geom;
  } frame_map_;

  rclcpp::Node::SharedPtr node_;
  ptree::PropertyClient property_client_;

  void defineConnections();

  void readFromVM(const view_model::JointViewModelPtr& joint);
  void readFromVM(const view_model::VisualViewModelPtr& visual);
  void readFromVM(const view_model::CollisionViewModelPtr& collision);
  void readFromVM(const view_model::InertialViewModelPtr& inertial);

  void readFromUI(const view_model::JointViewModelPtr& joint) const;
  void readFromUI(const view_model::VisualViewModelPtr& visual) const;
  void readFromUI(const view_model::CollisionViewModelPtr& collision) const;
  void readFromUI(const view_model::InertialViewModelPtr& inertial) const;

  void blockSignals(bool block);
  void emitChanged();

  static void arrangeVisualGeometryTypeFrames(const std::map<QString, QFrame*>& map, const QString& type);
};
}  // namespace ui
}  // namespace urdf_builder
