#pragma once

#include <map>

#include <QDialog>
#include <QListWidgetItem>

#include <tobas_property_client/property_client.hpp>

#include "../view_model/joint_view_model.hpp"
#include "../view_model/link_view_model.hpp"

namespace Ui
{
class UpdateLinkDialogUI;
using UpdateLinkDialogUIPtr = std::shared_ptr<UpdateLinkDialogUI>;
}  // namespace Ui

namespace tobas
{
namespace gui
{
namespace ub
{
namespace ui
{
class UrdfBuilderPanel;

class UpdateLinkDialog : public QDialog
{
  Q_OBJECT

  using self = UpdateLinkDialog;
  using super = QDialog;

  static constexpr char kConfigKey_VisualGeometryMeshBrowseDir[] = "visual_geometry_mesh_dir";
  static constexpr char kConfigKey_CollisionGeometryMeshBrowseDir[] = "collision_geometry_mesh_dir";

Q_SIGNALS:
  void Changed();

public:
  explicit UpdateLinkDialog(rclcpp::Node::SharedPtr node, UrdfBuilderPanel* main);

  void done(int code) override;

  void readFromVM(const view_model::LinkViewModelPtr& link_vm);
  void setTabsEnabled(bool enabled);

  const view_model::LinkViewModelPtr& viewModel() const;

private Q_SLOTS:
  void onVisualGeometryTypeComboBoxIndexChanged(int index);
  void onCollisionGeometryTypeComboBoxIndexChanged(int index);
  void onJointParentComboBoxIndexChanged(int index);
  void onJointTypeComboBoxIndexChanged(int index);
  void onJointSpinBoxValueChanged(double);
  void onVisualSpinBoxValueChanged(double);
  void onCollisionSpinBoxValueChanged(double);
  void onInertialSpinBoxValueChanged(double);
  void onLinkNameLineEditTextChanged(const QString& text);
  void onJointNameLineEditTextChanged(const QString& text);
  void onVisualNameLineEditTextChanged(const QString& text);
  void onVisualGeometryMeshPathLineEditTextChanged(const QString& text);
  void onCollisionNameLineEditTextChanged(const QString& text);
  void onCollisionGeometryMeshPathEditTextChanged(const QString& text);
  void onMaterialNameLineEditTextChanged(const QString& text);
  void onMaterialTexturePathLineEditTextChanged(const QString& text);
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
  const rclcpp::Node::SharedPtr node_;
  ptree::PropertyClient property_client_;

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
}  // namespace ub
}  // namespace gui
}  // namespace tobas
