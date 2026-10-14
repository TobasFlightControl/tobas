// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/ui/update_link_dialog.hpp"

#include <cassert>

#include <QColorDialog>
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>

#include "ui_update_link_dialog.h"

#include "tobas_urdf_builder_plugin/ui/double_map_input_dialog.hpp"
#include "tobas_urdf_builder_plugin/ui/string_input_dialog.hpp"
#include "tobas_urdf_builder_plugin/ui/urdf_builder_panel.hpp"
#include "tobas_urdf_builder_plugin/utils/constants.hpp"
#include "tobas_urdf_builder_plugin/utils/widget_item.hpp"

namespace tobas::gui::ub::ui
{
UpdateLinkDialog::UpdateLinkDialog(UrdfBuilderPanel* _main)
  : super(_main)
  , settings_store_("tobas", "urdf_builder")
  , main_(_main)
  , ui_(new Ui::UpdateLinkDialogUI())
  , link_vm_(new view_model::LinkViewModel())
{
  ui_->setupUi(this);

  // Hide unnecessary items.
  ui_->JointSafetyGroupBox->hide();
  ui_->JointCalibrationGroupBox->hide();
  ui_->JointMimicGroupBox->hide();

  frame_map_.visual_geom = {
    { "Box", ui_->VisualGeometryBoxTypeFrame },
    { "Cylinder", ui_->VisualGeometryCylinderTypeFrame },
    { "Sphere", ui_->VisualGeometrySphereTypeFrame },
    { "Mesh", ui_->VisualGeometryMeshTypeFrame },
  };

  frame_map_.collision_geom = {
    { "Box", ui_->CollisionGeometryBoxTypeFrame },
    { "Cylinder", ui_->CollisionGeometryCylinderTypeFrame },
    { "Sphere", ui_->CollisionGeometrySphereTypeFrame },
    { "Mesh", ui_->CollisionGeometryMeshTypeFrame },
  };

  readFromVM(link_vm_);
  defineConnections();
}

void UpdateLinkDialog::done(int _code)
{
  if (_code == Rejected) {
    super::done(_code);
    return;
  }

  if (ui_->LinkNameLineEdit->text().isEmpty()) {
    QMessageBox::warning(this, kError, "No name specified");
  }
  else {
    super::done(_code);
  }
}

void UpdateLinkDialog::readFromVM(const view_model::LinkViewModelPtr& _link_vm)
{
  link_vm_ = _link_vm->clone();

  blockSignals(true);

  ui_->VisualListWidget->clear();
  for (const auto& visual : link_vm_->visuals()) {
    ui_->VisualListWidget->addItem(new VisualListWidgetItem(visual));
  }

  ui_->CollisionListWidget->clear();
  for (const auto& collision : link_vm_->collisions()) {
    ui_->CollisionListWidget->addItem(new CollisionListWidgetItem(collision));
  }

  ui_->VisualOriginGroupBox->hide();
  ui_->VisualGeometryGroupBox->hide();
  ui_->VisualMaterialGroupBox->hide();
  ui_->CollisionOriginGroupBox->hide();
  ui_->CollisionGeometryGroupBox->hide();

  ui_->LinkNameLineEdit->setText(link_vm_->name());

  readFromVM(link_vm_->joint());
  readFromVM(link_vm_->inertial());

  blockSignals(false);
}

void UpdateLinkDialog::setTabsEnabled(bool _enabled)
{
  ui_->GeneralTab->setEnabled(_enabled);
  ui_->JointTab->setEnabled(_enabled);
  ui_->VisualTab->setEnabled(_enabled);
  ui_->CollisionTab->setEnabled(_enabled);
  ui_->InertialTab->setEnabled(_enabled);
}

const view_model::LinkViewModelPtr& UpdateLinkDialog::viewModel() const
{
  return link_vm_;
}

void UpdateLinkDialog::onVisualGeometryTypeComboBoxIndexChanged(int)
{
  qDebug() << "UpdateLinkDialog::onVisualGeometryTypeComboBoxIndexChanged";

  const auto geometry_type = ui_->VisualGeometryTypeComboBox->currentText();
  arrangeVisualGeometryTypeFrames(frame_map_.visual_geom, geometry_type);
  visual_vm_->geometry()->type(geometry_type);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::onCollisionGeometryTypeComboBoxIndexChanged(int)
{
  qDebug() << "UpdateLinkDialog::CollisionGeometryTypeComboBoxIndexChange";

  const auto geometry_type = ui_->CollisionGeometryTypeComboBox->currentText();
  arrangeVisualGeometryTypeFrames(frame_map_.collision_geom, geometry_type);
  collision_vm_->geometry()->type(geometry_type);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::onLinkNameLineEditTextChanged(const QString& _text)
{
  qDebug().nospace() << "UpdateLinkDialog::onLinkNameLineEditTextChanged(" << _text << ")";

  link_vm_->name(_text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::onJointNameLineEditTextChanged(const QString& _text)
{
  qDebug().nospace() << "UpdateLinkDialog::onJointNameLineEditTextChanged(" << _text << ")";

  link_vm_->joint()->name(_text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::onVisualNameLineEditTextChanged(const QString& _text)
{
  qDebug().nospace() << "UpdateLinkDialog::onVisualNameLineEditTextChanged(" << _text << ")";

  visual_vm_->name(_text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::onVisualGeometryMeshPathLineEditTextChanged(const QString& _text)
{
  qDebug().nospace() << "UpdateLinkDialog::onVisualGeometryMeshPathLineEditTextChanged(" << _text << ")";

  visual_vm_->geometry()->filePath(_text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::onCollisionNameLineEditTextChanged(const QString& _text)
{
  qDebug().nospace() << "UpdateLinkDialog::onCollisionNameLineEditTextChanged(" << _text << ")";

  collision_vm_->name(_text);

  emitChanged();
}

void UpdateLinkDialog::onCollisionGeometryMeshPathEditTextChanged(const QString& _text)
{
  qDebug().nospace() << "UpdateLinkDialog::onCollisionGeometryMeshPathEditTextChanged(" << _text << ")";

  collision_vm_->geometry()->filePath(_text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::onMaterialNameLineEditTextChanged(const QString& _text)
{
  qDebug().nospace() << "UpdateLinkDialog::onMaterialNameLineEditTextChanged(" << _text << ")";

  visual_vm_->material()->name(_text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::onMaterialTexturePathLineEditTextChanged(const QString& _text)
{
  qDebug().nospace() << "UpdateLinkDialog::onMaterialTexturePathLineEditTextChanged(" << _text << ")";

  visual_vm_->material()->textureFileName(_text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::onJointParentComboBoxIndexChanged(int)
{
  qDebug() << "UpdateLinkDialog::onJointParentComboBoxIndexChanged";

  readFromUI(link_vm_->joint());

  emitChanged();
}

void UpdateLinkDialog::onJointTypeComboBoxIndexChanged(int)
{
  qDebug() << "UpdateLinkDialog::onJointTypeComboBoxIndexChanged";

  readFromUI(link_vm_->joint());

  const auto& joint = link_vm_->joint();
  ui_->JointLimitGroupBox->setVisible(joint->limitsEnabled());

  // Do not show Axis or Dynamics for fixed joints.
  if (joint->isFixed()) {
    ui_->JointAxisGroupBox->setVisible(false);
    ui_->JointDynamicsGroupBox->setVisible(false);
  }
  else {
    ui_->JointAxisGroupBox->setVisible(true);
    ui_->JointDynamicsGroupBox->setVisible(true);
  }

  emitChanged();
}

void UpdateLinkDialog::onJointSpinBoxValueChanged(double)
{
  qDebug() << "UpdateLinkDialog::onJointSpinBoxValueChanged";

  readFromUI(link_vm_->joint());
  emitChanged();
}

void UpdateLinkDialog::onVisualSpinBoxValueChanged(double)
{
  qDebug() << "UpdateLinkDialog::onVisualSpinBoxValueChanged";

  readFromUI(visual_vm_);
  emitChanged();
}

void UpdateLinkDialog::onCollisionSpinBoxValueChanged(double)
{
  qDebug() << "UpdateLinkDialog::onCollisionSpinBoxValueChanged";

  readFromUI(collision_vm_);
  emitChanged();
}

void UpdateLinkDialog::onInertialSpinBoxValueChanged(double)
{
  qDebug() << "UpdateLinkDialog::onInertialSpinBoxValueChanged";

  readFromUI(link_vm_->inertial());
  emitChanged();
}

void UpdateLinkDialog::onVisualListWidgetItemClicked(QListWidgetItem* _item)
{
  qDebug().nospace() << "UpdateLinkDialog::onVisualListWidgetItemClicked(" << _item->text() << ")";

  const auto visual_item = dynamic_cast<VisualListWidgetItem*>(_item);
  readFromVM(visual_item->viewModel());
}

void UpdateLinkDialog::onCollisionListWidgetItemClicked(QListWidgetItem* _item)
{
  qDebug().nospace() << "UpdateLinkDialog::onCollisionListWidgetItemClicked(" << _item->text() << ")";

  const auto collision_item = dynamic_cast<CollisionListWidgetItem*>(_item);
  readFromVM(collision_item->viewModel());
}

void UpdateLinkDialog::onRenameLinkButtonClicked()
{
  qDebug() << "UpdateLinkDialog::onRenameLinkButtonClicked";

  const auto cur_name = ui_->LinkNameLineEdit->text();
  auto excludeds = main_->linkNames();
  excludeds.removeOne(cur_name);
  StringInputDialog dialog(this, "Rename Link", "Link Name", cur_name, excludeds);

  const auto result = dialog.exec();
  if (result != Accepted) {
    return;
  }

  ui_->LinkNameLineEdit->setText(dialog.getText());

  // FIXME: `TreeNode` is duplicated unless it is reloaded both when `LinkNameLineEdit` changes and here.
  emitChanged();
}

void UpdateLinkDialog::onRenameJointButtonClicked()
{
  qDebug() << "UpdateLinkDialog::onRenameJointButtonClicked";

  const auto cur_name = ui_->JointNameLineEdit->text();
  auto excludeds = main_->jointNames();
  excludeds.removeOne(cur_name);
  StringInputDialog dialog(this, "Rename Joint", "Joint Name", cur_name, excludeds);

  const auto result = dialog.exec();
  if (result != Accepted) {
    return;
  }

  ui_->JointNameLineEdit->setText(dialog.getText());

  emitChanged();
}

void UpdateLinkDialog::onAddVisualButtonClicked()
{
  qDebug() << "UpdateLinkDialog::onAddVisualButtonClicked";

  const auto visual_vm = std::make_shared<view_model::VisualViewModel>(nullptr);
  const auto item = new VisualListWidgetItem(visual_vm);
  ui_->VisualListWidget->addItem(item);
  ui_->VisualListWidget->setCurrentItem(item);
  link_vm_->add(visual_vm);
  readFromVM(visual_vm);

  emitChanged();
}

void UpdateLinkDialog::onRemoveVisualButtonClicked()
{
  qDebug() << "UpdateLinkDialog::onRemoveVisualButtonClicked";

  if (ui_->VisualListWidget->selectedItems().empty()) {
    QMessageBox::warning(this, kError, "No visual is selected.");
    return;
  }

  const auto item = ui_->VisualListWidget->selectedItems().front();
  const auto casted_item = dynamic_cast<VisualListWidgetItem*>(item);
  link_vm_->remove(casted_item->viewModel());
  ui_->VisualListWidget->removeItemWidget(item);
  delete item;

  if (ui_->VisualListWidget->count() > 0) {
    const auto first = dynamic_cast<VisualListWidgetItem*>(ui_->VisualListWidget->item(0));
    first->setSelected(true);
    readFromVM(first->viewModel());
  }
  else {
    readFromVM(std::shared_ptr<view_model::VisualViewModel>(nullptr));
  }

  emitChanged();
}

void UpdateLinkDialog::onAddCollisionButtonClicked()
{
  qDebug() << "UpdateLinkDialog::onAddCollisionButtonClicked";

  const auto collision_vm = std::make_shared<view_model::CollisionViewModel>(nullptr);
  const auto item = new CollisionListWidgetItem(collision_vm);
  ui_->CollisionListWidget->addItem(item);
  ui_->CollisionListWidget->setCurrentItem(item);
  link_vm_->add(collision_vm);
  readFromVM(collision_vm);

  emitChanged();
}

void UpdateLinkDialog::onRemoveCollisionButtonClicked()
{
  qDebug() << "UpdateLinkDialog::onRemoveCollisionButtonClicked";

  if (ui_->CollisionListWidget->selectedItems().empty()) {
    QMessageBox::warning(this, kError, "No collision is selected.");
    return;
  }

  const auto item = ui_->CollisionListWidget->selectedItems().front();
  const auto casted_item = dynamic_cast<CollisionListWidgetItem*>(item);
  link_vm_->remove(casted_item->viewModel());
  ui_->CollisionListWidget->removeItemWidget(item);
  delete item;

  if (ui_->CollisionListWidget->count() > 0) {
    const auto first = dynamic_cast<CollisionListWidgetItem*>(ui_->CollisionListWidget->item(0));
    first->setSelected(true);
    readFromVM(first->viewModel());
  }
  else {
    readFromVM(std::shared_ptr<view_model::CollisionViewModel>(nullptr));
  }

  emitChanged();
}

void UpdateLinkDialog::onVisualGeometryMeshBrowseButtonClicked()
{
  constexpr char kConfigKey_VisualGeometryMeshBrowseDir[] = "update_link_dialog/visual_geometry_mesh_dir";

  qDebug() << "UpdateLinkDialog::onVisualGeometryMeshBrowseButtonClicked";

  // Get the last opened directory.
  const auto last_dir = settings_store_.value(kConfigKey_VisualGeometryMeshBrowseDir, QDir::homePath()).toString();

  // Get the mesh file path.
  const auto file_path =
    QFileDialog::getOpenFileName(this, "Select Mesh File", last_dir, "Mesh Files (*.stl *.dae);;All Files (*)");
  if (file_path.isEmpty()) {
    return;
  }

  // Set the mesh file path.
  ui_->VisualGeometryMeshPathLineEdit->setText("file://" + file_path);

  // Save the last opened directory.
  const auto new_dir = QFileInfo(file_path).absolutePath();
  settings_store_.setValue(kConfigKey_VisualGeometryMeshBrowseDir, new_dir);
}

void UpdateLinkDialog::onCollisionGeometryMeshBrowseButtonClicked()
{
  constexpr char kConfigKey_CollisionGeometryMeshBrowseDir[] = "update_link_dialog/collision_geometry_mesh_dir";

  qDebug() << "UpdateLinkDialog::onCollisionGeometryMeshBrowseButtonClicked";

  // Get the last opened directory.
  const auto last_dir = settings_store_.value(kConfigKey_CollisionGeometryMeshBrowseDir, QDir::homePath()).toString();

  // Get the mesh file path.
  const auto file_path = QFileDialog::getOpenFileName(
    this,
    "Select Mesh File",
    last_dir,
    "Mesh Files (*.stl *.dae);;All Files (*)",
    nullptr,
    QFileDialog::DontUseNativeDialog);
  if (file_path.isEmpty()) {
    return;
  }

  // Set the mesh file path.
  ui_->CollisionGeometryMeshPathLineEdit->setText("file://" + file_path);

  // Save the last opened directory.
  const auto new_dir = QFileInfo(file_path).absolutePath();
  settings_store_.setValue(kConfigKey_CollisionGeometryMeshBrowseDir, new_dir);
}

void UpdateLinkDialog::onMaterialColorPickButtonClicked()
{
  qDebug() << "UpdateLinkDialog::onMaterialColorPickButtonClicked";

  const auto& color = visual_vm_->material()->color();
  QColorDialog dialog(QColor::fromRgbF(color.r, color.g, color.b, color.a));

  if (dialog.exec() != Accepted) {
    return;
  }

  visual_vm_->material()->color(dialog.currentColor());
  link_vm_->sync();
  readFromVM(visual_vm_);

  emitChanged();
}

void UpdateLinkDialog::onBuildInertiaBoxButtonClicked()
{
  qDebug() << "UpdateLinkDialog::onBuildInertiaBoxButtonClicked";

  DoubleMapInputDialog dialog(this, "Box Inertia", { "X", "Y", "Z" });
  const auto result = dialog.exec();

  if (result != Accepted) {
    return;
  }

  const auto& x = dialog.getValue("X");
  const auto& y = dialog.getValue("Y");
  const auto& z = dialog.getValue("Z");

  link_vm_->inertial()->buildInertiaBox(x, y, z);
  link_vm_->sync();
  readFromVM(link_vm_->inertial());

  emitChanged();
}

void UpdateLinkDialog::onBuildInertiaCylinderButtonClicked()
{
  qDebug() << "UpdateLinkDialog::onBuildInertiaCylinderButtonClicked";

  DoubleMapInputDialog dialog(this, "Cylinder Inertia", { "Radius", "Length" });
  const auto result = dialog.exec();

  if (result != Accepted) {
    return;
  }

  const auto& radius = dialog.getValue("Radius");
  const auto& length = dialog.getValue("Length");

  link_vm_->inertial()->buildInertiaCylinder(radius, length);
  link_vm_->sync();
  readFromVM(link_vm_->inertial());

  emitChanged();
}

void UpdateLinkDialog::onBuildInertiaSphereButtonClicked()
{
  qDebug() << "UpdateLinkDialog::onBuildInertiaSphereButtonClicked";

  DoubleMapInputDialog dialog(this, "Sphere Inertia", { "Radius" });
  const auto result = dialog.exec();

  if (result != Accepted) {
    return;
  }

  const auto& radius = dialog.getValue("Radius");

  link_vm_->inertial()->buildInertiaSphere(radius);
  link_vm_->sync();
  readFromVM(link_vm_->inertial());

  emitChanged();
}

void UpdateLinkDialog::readFromVM(const view_model::JointViewModelPtr& _joint)
{
  blockSignals(true);

  ui_->JointNameLineEdit->setText(_joint->name());

  ui_->JointParentLinkComboBox->clear();
  for (const auto& link_name : main_->linkNames()) {
    if (link_name != link_vm_->name()) {
      ui_->JointParentLinkComboBox->addItem(link_name);
    }
  }
  if (_joint->parentLinkName().isEmpty()) {
    ui_->JointParentLinkComboBox->setCurrentIndex(-1);
  }
  else {
    ui_->JointParentLinkComboBox->setCurrentText(_joint->parentLinkName());
  }

  ui_->JointTypeComboBox->setCurrentText(_joint->type());

  ui_->JointLimitGroupBox->setVisible(_joint->limitsEnabled());
  if (_joint->limitsEnabled()) {
    ui_->JointLimitLowerSpinBox->setValue(_joint->limits()->lower());
    ui_->JointLimitUpperSpinBox->setValue(_joint->limits()->upper());
    ui_->JointLimitEffortSpinBox->setValue(_joint->limits()->effort());
    ui_->JointLimitVelocitySpinBox->setValue(_joint->limits()->velocity());
  }

  // Do not show Axis or Dynamics for fixed joints.
  if (_joint->isFixed()) {
    ui_->JointAxisGroupBox->setVisible(false);
    ui_->JointDynamicsGroupBox->setVisible(false);
  }
  else {
    ui_->JointAxisGroupBox->setVisible(true);
    ui_->JointDynamicsGroupBox->setVisible(true);
  }

  const auto& origin = _joint->origin();
  ui_->JointOriginXSpinBox->setValue(origin.position.x);
  ui_->JointOriginYSpinBox->setValue(origin.position.y);
  ui_->JointOriginZSpinBox->setValue(origin.position.z);

  double r, p, y;
  origin.rotation.getRPY(r, p, y);
  ui_->JointOriginRollSpinBox->setValue(r);
  ui_->JointOriginPitchSpinBox->setValue(p);
  ui_->JointOriginYawSpinBox->setValue(y);

  ui_->JointAxisXSpinBox->setValue(_joint->axis().x);
  ui_->JointAxisYSpinBox->setValue(_joint->axis().y);
  ui_->JointAxisZSpinBox->setValue(_joint->axis().z);

  blockSignals(false);
}

void UpdateLinkDialog::readFromVM(const view_model::VisualViewModelPtr& _visual)
{
  visual_vm_ = _visual;
  ui_->VisualOriginGroupBox->setVisible(_visual != nullptr);
  ui_->VisualGeometryGroupBox->setVisible(_visual != nullptr);
  ui_->VisualMaterialGroupBox->setVisible(_visual != nullptr);

  if (!_visual) {
    return;
  }

  blockSignals(true);

  ui_->VisualNameLineEdit->setText(_visual->name());
  ui_->VisualGeometryTypeComboBox->setCurrentText(visual_vm_->geometry()->name());

  ui_->VisualOriginXSpinBox->setValue(visual_vm_->origin().position.x);
  ui_->VisualOriginYSpinBox->setValue(visual_vm_->origin().position.y);
  ui_->VisualOriginZSpinBox->setValue(visual_vm_->origin().position.z);

  double r, p, y;
  visual_vm_->origin().rotation.getRPY(r, p, y);
  ui_->VisualOriginRollSpinBox->setValue(r);
  ui_->VisualOriginPitchSpinBox->setValue(p);
  ui_->VisualOriginYawSpinBox->setValue(y);

  const auto& geometry_vm = visual_vm_->geometry();
  ui_->VisualGeometryBoxLengthSpinBox->setValue(geometry_vm->length());
  ui_->VisualGeometryBoxWidthSpinBox->setValue(geometry_vm->width());
  ui_->VisualGeometryBoxHeightSpinBox->setValue(geometry_vm->height());
  ui_->VisualGeometryCylinderLengthSpinBox->setValue(geometry_vm->length());
  ui_->VisualGeometryCylinderRadiusSpinBox->setValue(geometry_vm->radius());
  ui_->VisualGeometrySphereRadiusSpinBox->setValue(geometry_vm->radius());
  ui_->VisualGeometryMeshPathLineEdit->setText(geometry_vm->filePath());
  ui_->VisualGeometryMeshScaleSpinBox->setValue(geometry_vm->scale().x);

  const auto& material_vm = visual_vm_->material();
  ui_->MaterialNameLineEdit->setText(material_vm->name());
  ui_->MaterialColorRedSpinBox->setValue(material_vm->color().r);
  ui_->MaterialColorGreenSpinBox->setValue(material_vm->color().g);
  ui_->MaterialColorBlueSpinBox->setValue(material_vm->color().b);
  ui_->MaterialTexturePathLineEdit->setText(material_vm->textureFileName());

  arrangeVisualGeometryTypeFrames(frame_map_.visual_geom, ui_->VisualGeometryTypeComboBox->currentText());

  blockSignals(false);
}

void UpdateLinkDialog::readFromVM(const view_model::CollisionViewModelPtr& _collision)
{
  collision_vm_ = _collision;
  ui_->CollisionOriginGroupBox->setVisible(_collision != nullptr);
  ui_->CollisionGeometryGroupBox->setVisible(_collision != nullptr);

  if (!_collision) {
    return;
  }

  blockSignals(true);

  ui_->CollisionNameLineEdit->setText(_collision->name());
  ui_->CollisionGeometryTypeComboBox->setCurrentText(collision_vm_->geometry()->name());
  ui_->CollisionOriginXSpinBox->setValue(collision_vm_->origin().position.x);
  ui_->CollisionOriginYSpinBox->setValue(collision_vm_->origin().position.y);
  ui_->CollisionOriginZSpinBox->setValue(collision_vm_->origin().position.z);

  double r, p, y;
  collision_vm_->origin().rotation.getRPY(r, p, y);
  ui_->CollisionOriginRollSpinBox->setValue(r);
  ui_->CollisionOriginPitchSpinBox->setValue(p);
  ui_->CollisionOriginYawSpinBox->setValue(y);

  const auto& geometry_vm = collision_vm_->geometry();
  ui_->CollisionGeometryBoxLengthSpinBox->setValue(geometry_vm->length());
  ui_->CollisionGeometryBoxWidthSpinBox->setValue(geometry_vm->width());
  ui_->CollisionGeometryBoxHeightSpinBox->setValue(geometry_vm->height());
  ui_->CollisionGeometryCylinderLengthSpinBox->setValue(geometry_vm->length());
  ui_->CollisionGeometryCylinderRadiusSpinBox->setValue(geometry_vm->radius());
  ui_->CollisionGeometrySphereRadiusSpinBox->setValue(geometry_vm->radius());
  ui_->CollisionGeometryMeshPathLineEdit->setText(geometry_vm->filePath());
  ui_->CollisionGeometryMeshScaleSpinBox->setValue(geometry_vm->scale().x);

  arrangeVisualGeometryTypeFrames(frame_map_.collision_geom, ui_->CollisionGeometryTypeComboBox->currentText());

  blockSignals(false);
}

void UpdateLinkDialog::readFromVM(const view_model::InertialViewModelPtr& _inertial)
{
  blockSignals(true);

  const auto& origin = _inertial->origin();
  ui_->InertialOriginXSpinBox->setValue(origin.position.x);
  ui_->InertialOriginYSpinBox->setValue(origin.position.y);
  ui_->InertialOriginZSpinBox->setValue(origin.position.z);

  double r, p, y;
  origin.rotation.getRPY(r, p, y);
  ui_->InertialOriginRollSpinBox->setValue(r);
  ui_->InertialOriginPitchSpinBox->setValue(p);
  ui_->InertialOriginYawSpinBox->setValue(y);

  ui_->InertialMassSpinBox->setValue(link_vm_->inertial()->mass());

  const auto& inertia = _inertial->inertia();
  ui_->InertiaIXXSpinBox->setValue(inertia.ixx);
  ui_->InertiaIXYSpinBox->setValue(inertia.ixy);
  ui_->InertiaIXZSpinBox->setValue(inertia.ixz);
  ui_->InertiaIYYSpinBox->setValue(inertia.iyy);
  ui_->InertiaIYZSpinBox->setValue(inertia.iyz);
  ui_->InertiaIZZSpinBox->setValue(inertia.izz);

  blockSignals(false);
}

void UpdateLinkDialog::readFromUI(const view_model::VisualViewModelPtr& _visual) const
{
  assert(_visual);

  ::urdf::Pose pose;
  pose.position.x = ui_->VisualOriginXSpinBox->value();
  pose.position.y = ui_->VisualOriginYSpinBox->value();
  pose.position.z = ui_->VisualOriginZSpinBox->value();
  pose.rotation.setFromRPY(
    ui_->VisualOriginRollSpinBox->value(), ui_->VisualOriginPitchSpinBox->value(), ui_->VisualOriginYawSpinBox->value());
  _visual->origin(pose);

  const auto& geometry_vm = _visual->geometry();
  switch (geometry_vm->type()) {
    case GeometryType::BOX:
      geometry_vm->length(ui_->VisualGeometryBoxLengthSpinBox->value());
      geometry_vm->width(ui_->VisualGeometryBoxWidthSpinBox->value());
      geometry_vm->height(ui_->VisualGeometryBoxHeightSpinBox->value());
      break;
    case GeometryType::SPHERE:
      geometry_vm->radius(ui_->VisualGeometrySphereRadiusSpinBox->value());
      break;
    case GeometryType::CYLINDER:
      geometry_vm->radius(ui_->VisualGeometryCylinderRadiusSpinBox->value());
      geometry_vm->length(ui_->VisualGeometryCylinderLengthSpinBox->value());
      break;
    case GeometryType::MESH:
      geometry_vm->filePath(ui_->VisualGeometryMeshPathLineEdit->text());
      const auto scale = ui_->VisualGeometryMeshScaleSpinBox->value();
      geometry_vm->scale(::urdf::Vector3(scale, scale, scale));
      break;
  }

  const auto& material_vm = visual_vm_->material();
  material_vm->name(ui_->MaterialNameLineEdit->text());
  material_vm->color(
    ui_->MaterialColorRedSpinBox->value(),
    ui_->MaterialColorGreenSpinBox->value(),
    ui_->MaterialColorBlueSpinBox->value());
  material_vm->textureFileName(ui_->MaterialTexturePathLineEdit->text());
  link_vm_->sync();
}

void UpdateLinkDialog::readFromUI(const view_model::CollisionViewModelPtr& _collision) const
{
  assert(_collision);

  ::urdf::Pose pose;
  pose.position.x = ui_->CollisionOriginXSpinBox->value();
  pose.position.y = ui_->CollisionOriginYSpinBox->value();
  pose.position.z = ui_->CollisionOriginZSpinBox->value();
  pose.rotation.setFromRPY(
    ui_->CollisionOriginRollSpinBox->value(),
    ui_->CollisionOriginPitchSpinBox->value(),
    ui_->CollisionOriginYawSpinBox->value());
  _collision->origin(pose);

  const auto& geometry_vm = _collision->geometry();
  switch (geometry_vm->type()) {
    case GeometryType::BOX:
      geometry_vm->length(ui_->CollisionGeometryBoxLengthSpinBox->value());
      geometry_vm->width(ui_->CollisionGeometryBoxWidthSpinBox->value());
      geometry_vm->height(ui_->CollisionGeometryBoxHeightSpinBox->value());
      break;
    case GeometryType::SPHERE:
      geometry_vm->radius(ui_->CollisionGeometrySphereRadiusSpinBox->value());
      break;
    case GeometryType::CYLINDER:
      geometry_vm->radius(ui_->CollisionGeometryCylinderRadiusSpinBox->value());
      geometry_vm->length(ui_->CollisionGeometryCylinderLengthSpinBox->value());
      break;
    case GeometryType::MESH:
      geometry_vm->filePath(ui_->CollisionGeometryMeshPathLineEdit->text());
      const auto scale = ui_->CollisionGeometryMeshScaleSpinBox->value();
      geometry_vm->scale(::urdf::Vector3(scale, scale, scale));
      break;
  }
  link_vm_->sync();
}

void UpdateLinkDialog::readFromUI(const view_model::JointViewModelPtr& _joint) const
{
  _joint->name(ui_->JointNameLineEdit->text());
  _joint->parentLinkName(ui_->JointParentLinkComboBox->currentText());
  _joint->childLinkName(ui_->LinkNameLineEdit->text());
  _joint->type(ui_->JointTypeComboBox->currentText());

  if (_joint->limitsEnabled()) {
    _joint->limits()->lower(ui_->JointLimitLowerSpinBox->value());
    _joint->limits()->upper(ui_->JointLimitUpperSpinBox->value());
    _joint->limits()->effort(ui_->JointLimitEffortSpinBox->value());
    _joint->limits()->velocity(ui_->JointLimitVelocitySpinBox->value());
  }

  ::urdf::Pose pose;
  pose.position.x = ui_->JointOriginXSpinBox->value();
  pose.position.y = ui_->JointOriginYSpinBox->value();
  pose.position.z = ui_->JointOriginZSpinBox->value();
  pose.rotation.setFromRPY(
    ui_->JointOriginRollSpinBox->value(), ui_->JointOriginPitchSpinBox->value(), ui_->JointOriginYawSpinBox->value());
  _joint->origin(pose);

  ::urdf::Vector3 axis;
  axis.x = ui_->JointAxisXSpinBox->value();
  axis.y = ui_->JointAxisYSpinBox->value();
  axis.z = ui_->JointAxisZSpinBox->value();
  _joint->axis(axis);

  link_vm_->sync();
}

void UpdateLinkDialog::readFromUI(const view_model::InertialViewModelPtr& _inertial) const
{
  ::urdf::Pose pose;
  pose.position.x = ui_->InertialOriginXSpinBox->value();
  pose.position.y = ui_->InertialOriginYSpinBox->value();
  pose.position.z = ui_->InertialOriginZSpinBox->value();
  pose.rotation.setFromRPY(
    ui_->InertialOriginRollSpinBox->value(),
    ui_->InertialOriginPitchSpinBox->value(),
    ui_->InertialOriginYawSpinBox->value());
  _inertial->origin(pose);

  _inertial->mass(ui_->InertialMassSpinBox->value());

  view_model::Inertia inertia;
  inertia.ixx = ui_->InertiaIXXSpinBox->value();
  inertia.ixy = ui_->InertiaIXYSpinBox->value();
  inertia.ixz = ui_->InertiaIXZSpinBox->value();
  inertia.iyy = ui_->InertiaIYYSpinBox->value();
  inertia.iyz = ui_->InertiaIYZSpinBox->value();
  inertia.izz = ui_->InertiaIZZSpinBox->value();
  _inertial->inertia(inertia);

  link_vm_->sync();
}

void UpdateLinkDialog::blockSignals(bool _block)
{
  const auto widget_list = findChildren<QWidget*>();
  const QList<QWidget*>::const_iterator last_widget(widget_list.end());
  QList<QWidget*>::const_iterator widget_iter(widget_list.begin());

  while (widget_iter != last_widget) {
    (*widget_iter)->blockSignals(_block);
    ++widget_iter;
  }
}

void UpdateLinkDialog::emitChanged()
{
  Q_EMIT Changed();
}

void UpdateLinkDialog::arrangeVisualGeometryTypeFrames(const std::map<QString, QFrame*>& _map, const QString& _type)
{
  for (const auto& pair : _map) {
    pair.second->hide();
  }
  _map.at(_type)->show();
}
}  // namespace tobas::gui::ub::ui
