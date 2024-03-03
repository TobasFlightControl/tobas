#include <ros/ros.h>
#include <QMessageBox>
#include <QColorDialog>
#include <QFileDialog>
#include <QDir>

#include "../../include/urdf_builder/ui/update_link_dialog.hpp"
#include "../../include/urdf_builder/ui/widget_item.hpp"
#include "../../include/urdf_builder/ui/double_map_input_dialog.hpp"
#include "../../include/urdf_builder/ui/string_input_dialog.hpp"
#include "../../include/urdf_builder/utils/constants.hpp"
#include "ui_update_link_dialog.h"

using namespace std;

namespace urdf_builder
{
namespace ui
{
UpdateLinkDialog::UpdateLinkDialog(const view_model::LinkViewModelPtr& vm, QWidget* parent)
  : QDialog(parent), ui_(new Ui::UpdateLinkDialogUI()), vm_(vm)
{
  ui_->setupUi(this);

  // 不要な項目を不可視にする
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

  readFromVM(vm);
  defineConnections();
}

void UpdateLinkDialog::done(int code)
{
  if (code == QDialog::Rejected)
  {
    QDialog::done(code);
    return;
  }

  if (ui_->LinkNameLineEdit->text().isEmpty())
    QMessageBox::warning(this, kError, "No name specified");
  else
    QDialog::done(code);
}

void UpdateLinkDialog::readFromVM(const view_model::LinkViewModelPtr& vm)
{
  vm_ = vm;

  blockSignals(true);

  ui_->VisualListWidget->clear();
  for (const auto& visual : vm_->visuals())
    ui_->VisualListWidget->addItem(new VisualListWidgetItem(visual));

  ui_->CollisionListWidget->clear();
  for (const auto& collision : vm_->collisions())
    ui_->CollisionListWidget->addItem(new CollisionListWidgetItem(collision));

  ui_->VisualOriginGroupBox->hide();
  ui_->VisualGeometryGroupBox->hide();
  ui_->VisualMaterialGroupBox->hide();
  ui_->CollisionOriginGroupBox->hide();
  ui_->CollisionGeometryGroupBox->hide();

  ui_->LinkNameLineEdit->setText(vm_->name());

  readFromVM(vm_->joint());
  readFromVM(vm_->inertial());

  blockSignals(false);
}

void UpdateLinkDialog::setTabsEnabled(bool enabled)
{
  ui_->GeneralTab->setEnabled(enabled);
  ui_->JointTab->setEnabled(enabled);
  ui_->VisualTab->setEnabled(enabled);
  ui_->CollisionTab->setEnabled(enabled);
  ui_->InertialTab->setEnabled(enabled);
}

const view_model::LinkViewModelPtr& UpdateLinkDialog::viewModel() const
{
  return vm_;
}

void UpdateLinkDialog::VisualGeometryTypeComboBoxIndexChanged(int)
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::VisualGeometryTypeComboBoxIndexChanged");

  const auto& cb = ui_->VisualGeometryTypeComboBox;
  arrangeVisualGeometryTypeFrames(frame_map_.visual_geom, cb->currentText());
  vvm_->geometry()->type(cb->currentText());
  vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::CollisionGeometryTypeComboBoxIndexChanged(int)
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::CollisionGeometryTypeComboBoxIndexChange");

  const auto& cb = ui_->CollisionGeometryTypeComboBox;
  arrangeVisualGeometryTypeFrames(frame_map_.collision_geom, cb->currentText());
  cvm_->geometry()->type(cb->currentText());
  vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::LinkNameLineEditTextChanged(const QString& text)
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::LinkNameLineEditTextChanged(" << text.toStdString() << ")");

  vm_->name(text);
  vm_->joint()->childLinkName(text);
  vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::JointNameLineEditTextChanged(const QString& text)
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::JointNameLineEditTextChanged(" << text.toStdString() << ")");

  vm_->joint()->name(text);
  vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::VisualNameLineEditTextChanged(const QString& text)
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::VisualNameLineEditTextChanged(" << text.toStdString() << ")");

  vvm_->name(text);
  vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::VisualGeometryMeshPathLineEditTextChanged(const QString& text)
{
  ROS_DEBUG_STREAM(
    "UpdateLinkDialog::VisualGeometryMeshPathLineEditTextChanged(" << text.toStdString() << ")");

  vvm_->geometry()->filePath(text);
  vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::CollisionNameLineEditTextChanged(const QString& text)
{
  ROS_DEBUG_STREAM(
    "UpdateLinkDialog::CollisionNameLineEditTextChanged(" << text.toStdString() << ")");

  cvm_->name(text);

  emitChanged();
}

void UpdateLinkDialog::CollisionGeometryMeshPathEditTextChanged(const QString& text)
{
  ROS_DEBUG_STREAM(
    "UpdateLinkDialog::CollisionGeometryMeshPathEditTextChanged(" << text.toStdString() << ")");

  cvm_->geometry()->filePath(text);
  vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::MaterialNameLineEditTextChanged(const QString& text)
{
  ROS_DEBUG_STREAM(
    "UpdateLinkDialog::MaterialNameLineEditTextChanged(" << text.toStdString() << ")");

  vvm_->material()->name(text);
  vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::MaterialTexturePathLineEditTextChanged(const QString& text)
{
  ROS_DEBUG_STREAM(
    "UpdateLinkDialog::MaterialTexturePathLineEditTextChanged(" << text.toStdString() << ")");

  vvm_->material()->textureFileName(text);
  vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::JointParentComboBoxIndexChanged(int)
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::JointParentComboBoxIndexChanged");

  readFromUI(vm_->joint());

  emitChanged();
}

void UpdateLinkDialog::JointTypeComboBoxIndexChanged(int)
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::JointTypeComboBoxIndexChanged");

  readFromUI(vm_->joint());

  const auto& joint = vm_->joint();
  ui_->JointLimitGroupBox->setVisible(joint->limitsEnabled());

  // 固定関節ならばAxis, Dynamicsは表示しない
  if (joint->isFixed())
  {
    ui_->JointAxisGroupBox->setVisible(false);
    ui_->JointDynamicsGroupBox->setVisible(false);
  }
  else
  {
    ui_->JointAxisGroupBox->setVisible(true);
    ui_->JointDynamicsGroupBox->setVisible(true);
  }

  emitChanged();
}

void UpdateLinkDialog::JointSpinBoxValueChanged(double)
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::JointSpinBoxValueChanged");

  readFromUI(vm_->joint());
  emitChanged();
}

void UpdateLinkDialog::VisualSpinBoxValueChanged(double)
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::VisualSpinBoxValueChanged");

  readFromUI(vvm_);
  emitChanged();
}

void UpdateLinkDialog::CollisionSpinBoxValueChanged(double)
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::CollisionSpinBoxValueChanged");

  readFromUI(cvm_);
  emitChanged();
}

void UpdateLinkDialog::InertialSpinBoxValueChanged(double)
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::InertialSpinBoxValueChanged");

  readFromUI(vm_->inertial());
  emitChanged();
}

void UpdateLinkDialog::VisualListWidgetItemClicked(QListWidgetItem* item)
{
  ROS_DEBUG_STREAM(
    "UpdateLinkDialog::VisualListWidgetItemClicked(" << item->text().toStdString() << ")");

  auto visualItem = dynamic_cast<VisualListWidgetItem*>(item);
  readFromVM(visualItem->viewModel());
}

void UpdateLinkDialog::CollisionListWidgetItemClicked(QListWidgetItem* item)
{
  ROS_DEBUG_STREAM(
    "UpdateLinkDialog::CollisionListWidgetItemClicked(" << item->text().toStdString() << ")");

  const auto collisionItem = dynamic_cast<CollisionListWidgetItem*>(item);
  readFromVM(collisionItem->viewModel());
}

void UpdateLinkDialog::RenameLinkButtonClicked()
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::RenameLinkButtonClicked");

  const auto& cur_name = ui_->LinkNameLineEdit->text();
  auto excludeds = vm_->usedLinkNames();
  excludeds.removeOne(cur_name);
  StringInputDialog dialog("Rename Link", "Link Name", cur_name, excludeds);

  const auto result = dialog.exec();
  if (result != QDialog::Accepted)
    return;

  ui_->LinkNameLineEdit->setText(dialog.getText());

  // FIXME: LinkNameLineEditの変更時とここで2回リロードしないとTreeNodeが重複する
  emitChanged();
}

void UpdateLinkDialog::RenameJointButtonClicked()
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::RenameJointButtonClicked");

  StringInputDialog dialog("Rename Joint", "Joint Name", ui_->JointNameLineEdit->text());

  const auto result = dialog.exec();
  if (result != QDialog::Accepted)
    return;

  ui_->JointNameLineEdit->setText(dialog.getText());

  emitChanged();
}

void UpdateLinkDialog::AddVisualButtonClicked()
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::AddVisualButtonClicked");

  const auto& vm = make_shared<view_model::VisualViewModel>(nullptr);
  const auto item = new VisualListWidgetItem(vm);
  ui_->VisualListWidget->addItem(item);
  ui_->VisualListWidget->setCurrentItem(item);
  vm_->add(vm);
  readFromVM(vm);

  emitChanged();
}

void UpdateLinkDialog::RemoveVisualButtonClicked()
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::RemoveVisualButtonClicked");

  if (ui_->VisualListWidget->selectedItems().empty())
  {
    QMessageBox::warning(this, kError, "No visual is selected.");
    return;
  }

  const auto item = ui_->VisualListWidget->selectedItems().front();
  const auto casted_item = dynamic_cast<VisualListWidgetItem*>(item);
  vm_->remove(casted_item->viewModel());
  ui_->VisualListWidget->removeItemWidget(item);
  delete item;

  if (ui_->VisualListWidget->count() > 0)
  {
    const auto first = dynamic_cast<VisualListWidgetItem*>(ui_->VisualListWidget->item(0));
    first->setSelected(true);
    readFromVM(first->viewModel());
  }
  else
  {
    readFromVM(shared_ptr<view_model::VisualViewModel>(nullptr));
  }

  emitChanged();
}

void UpdateLinkDialog::AddCollisionButtonClicked()
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::AddCollisionButtonClicked");

  const auto& vm = make_shared<view_model::CollisionViewModel>(nullptr);
  const auto item = new CollisionListWidgetItem(vm);
  ui_->CollisionListWidget->addItem(item);
  ui_->CollisionListWidget->setCurrentItem(item);
  vm_->add(vm);
  readFromVM(vm);

  emitChanged();
}

void UpdateLinkDialog::RemoveCollisionButtonClicked()
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::RemoveCollisionButtonClicked");

  if (ui_->CollisionListWidget->selectedItems().empty())
  {
    QMessageBox::warning(this, kError, "No collision is selected.");
    return;
  }

  const auto item = ui_->CollisionListWidget->selectedItems().front();
  const auto casted_item = dynamic_cast<CollisionListWidgetItem*>(item);
  vm_->remove(casted_item->viewModel());
  ui_->CollisionListWidget->removeItemWidget(item);
  delete item;

  if (ui_->CollisionListWidget->count() > 0)
  {
    const auto first = dynamic_cast<CollisionListWidgetItem*>(ui_->CollisionListWidget->item(0));
    first->setSelected(true);
    readFromVM(first->viewModel());
  }
  else
  {
    readFromVM(shared_ptr<view_model::CollisionViewModel>(nullptr));
  }

  emitChanged();
}

void UpdateLinkDialog::VisualGeometryMeshBrowseButtonClicked()
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::VisualGeometryMeshBrowseButtonClicked");

  const QString file_path = QFileDialog::getOpenFileName(
    this, tr("URDF Builder"), QDir::homePath(), tr("Mesh Files (*.stl *.dae);;All Files (*)"));

  if (file_path.isEmpty())
    return;

  ui_->VisualGeometryMeshPathLineEdit->setText("file://" + file_path);
}

void UpdateLinkDialog::CollisionGeometryMeshBrowseButtonClicked()
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::CollisionGeometryMeshBrowseButtonClicked");

  const QString file_path = QFileDialog::getOpenFileName(
    this, tr("URDF Builder"), QDir::homePath(), tr("Mesh Files (*.stl *.dae);;All Files (*)"));

  if (file_path.isEmpty())
    return;

  ui_->CollisionGeometryMeshPathLineEdit->setText("file://" + file_path);
}

void UpdateLinkDialog::MaterialColorPickButtonClicked()
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::MaterialColorPickButtonClicked");

  const auto& color = vvm_->material()->color();
  QColorDialog dialog(QColor::fromRgbF(color.r, color.g, color.b, color.a));

  if (dialog.exec() != QDialog::Accepted)
    return;

  vvm_->material()->color(dialog.currentColor());
  vm_->sync();
  readFromVM(vvm_);

  emitChanged();
}

void UpdateLinkDialog::BuildInertiaBoxButtonClicked()
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::BuildInertiaBoxButtonClicked");

  DoubleMapInputDialog dialog("Box Inertia", { "X", "Y", "Z" });
  const auto result = dialog.exec();

  if (result != QDialog::Accepted)
    return;

  const auto& x = dialog.getValue("X");
  const auto& y = dialog.getValue("Y");
  const auto& z = dialog.getValue("Z");

  vm_->inertial()->buildInertiaBox(x, y, z);
  vm_->sync();
  readFromVM(vm_->inertial());

  emitChanged();
}

void UpdateLinkDialog::BuildInertiaCylinderButtonClicked()
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::BuildInertiaCylinderButtonClicked");

  DoubleMapInputDialog dialog("Cylinder Inertia", { "Radius", "Length" });
  const auto result = dialog.exec();

  if (result != QDialog::Accepted)
    return;

  const auto& radius = dialog.getValue("Radius");
  const auto& length = dialog.getValue("Length");

  vm_->inertial()->buildInertiaCylinder(radius, length);
  vm_->sync();
  readFromVM(vm_->inertial());

  emitChanged();
}

void UpdateLinkDialog::BuildInertiaSphereButtonClicked()
{
  ROS_DEBUG_STREAM("UpdateLinkDialog::BuildInertiaSphereButtonClicked");

  DoubleMapInputDialog dialog("Sphere Inertia", { "Radius" });
  const auto result = dialog.exec();

  if (result != QDialog::Accepted)
    return;

  const auto& radius = dialog.getValue("Radius");

  vm_->inertial()->buildInertiaSphere(radius);
  vm_->sync();
  readFromVM(vm_->inertial());

  emitChanged();
}

void UpdateLinkDialog::readFromVM(const view_model::JointViewModelPtr& joint)
{
  blockSignals(true);

  ui_->JointNameLineEdit->setText(joint->name());

  ui_->JointParentLinkComboBox->clear();
  for (const auto& link_name : joint->usedLinkNames())
    if (link_name != vm_->name())
      ui_->JointParentLinkComboBox->addItem(link_name);
  if (joint->parentLinkName().isEmpty())
    ui_->JointParentLinkComboBox->setCurrentIndex(-1);
  else
    ui_->JointParentLinkComboBox->setCurrentText(joint->parentLinkName());

  ui_->JointTypeComboBox->setCurrentText(joint->type());

  ui_->JointLimitGroupBox->setVisible(joint->limitsEnabled());
  if (joint->limitsEnabled())
  {
    ui_->JointLimitLowerSpinBox->setValue(joint->limits()->lower());
    ui_->JointLimitUpperSpinBox->setValue(joint->limits()->upper());
    ui_->JointLimitEffortSpinBox->setValue(joint->limits()->effort());
    ui_->JointLimitVelocitySpinBox->setValue(joint->limits()->velocity());
  }

  // 固定関節ならばAxis, Dynamicsは表示しない
  if (joint->isFixed())
  {
    ui_->JointAxisGroupBox->setVisible(false);
    ui_->JointDynamicsGroupBox->setVisible(false);
  }
  else
  {
    ui_->JointAxisGroupBox->setVisible(true);
    ui_->JointDynamicsGroupBox->setVisible(true);
  }

  const auto& origin = joint->origin();
  ui_->JointOriginXSpinBox->setValue(origin.position.x);
  ui_->JointOriginYSpinBox->setValue(origin.position.y);
  ui_->JointOriginZSpinBox->setValue(origin.position.z);

  double r, p, y;
  origin.rotation.getRPY(r, p, y);
  ui_->JointOriginRollSpinBox->setValue(r);
  ui_->JointOriginPitchSpinBox->setValue(p);
  ui_->JointOriginYawSpinBox->setValue(y);

  ui_->JointAxisXSpinBox->setValue(joint->axis().x);
  ui_->JointAxisYSpinBox->setValue(joint->axis().y);
  ui_->JointAxisZSpinBox->setValue(joint->axis().z);

  blockSignals(false);
}

void UpdateLinkDialog::readFromVM(const view_model::VisualViewModelPtr& visual)
{
  vvm_ = visual;
  ui_->VisualOriginGroupBox->setVisible(visual != nullptr);
  ui_->VisualGeometryGroupBox->setVisible(visual != nullptr);
  ui_->VisualMaterialGroupBox->setVisible(visual != nullptr);

  if (!visual)
    return;

  blockSignals(true);

  ui_->VisualNameLineEdit->setText(visual->name());
  ui_->VisualGeometryTypeComboBox->setCurrentText(vvm_->geometry()->name());

  ui_->VisualOriginXSpinBox->setValue(vvm_->origin().position.x);
  ui_->VisualOriginYSpinBox->setValue(vvm_->origin().position.y);
  ui_->VisualOriginZSpinBox->setValue(vvm_->origin().position.z);

  double r, p, y;
  vvm_->origin().rotation.getRPY(r, p, y);
  ui_->VisualOriginRollSpinBox->setValue(r);
  ui_->VisualOriginPitchSpinBox->setValue(p);
  ui_->VisualOriginYawSpinBox->setValue(y);

  auto gvm = vvm_->geometry();
  ui_->VisualGeometryBoxLengthSpinBox->setValue(gvm->length());
  ui_->VisualGeometryBoxWidthSpinBox->setValue(gvm->width());
  ui_->VisualGeometryBoxHeightSpinBox->setValue(gvm->height());
  ui_->VisualGeometryCylinderLengthSpinBox->setValue(gvm->length());
  ui_->VisualGeometryCylinderRadiusSpinBox->setValue(gvm->radius());
  ui_->VisualGeometrySphereRadiusSpinBox->setValue(gvm->radius());
  ui_->VisualGeometryMeshPathLineEdit->setText(gvm->filePath());
  ui_->VisualGeometryMeshScaleSpinBox->setValue(gvm->scale().x);  // FIXME: scaleを3軸設定すべき？

  auto mvm = vvm_->material();
  ui_->MaterialNameLineEdit->setText(mvm->name());
  ui_->MaterialColorRedSpinBox->setValue(mvm->color().r);
  ui_->MaterialColorGreenSpinBox->setValue(mvm->color().g);
  ui_->MaterialColorBlueSpinBox->setValue(mvm->color().b);
  ui_->MaterialTexturePathLineEdit->setText(mvm->textureFileName());

  arrangeVisualGeometryTypeFrames(
    frame_map_.visual_geom, ui_->VisualGeometryTypeComboBox->currentText());

  blockSignals(false);
}

void UpdateLinkDialog::readFromVM(const view_model::CollisionViewModelPtr& collision)
{
  cvm_ = collision;
  ui_->CollisionOriginGroupBox->setVisible(collision != nullptr);
  ui_->CollisionGeometryGroupBox->setVisible(collision != nullptr);

  if (!collision)
    return;

  blockSignals(true);

  ui_->CollisionNameLineEdit->setText(collision->name());
  ui_->CollisionGeometryTypeComboBox->setCurrentText(cvm_->geometry()->name());
  ui_->CollisionOriginXSpinBox->setValue(cvm_->origin().position.x);
  ui_->CollisionOriginYSpinBox->setValue(cvm_->origin().position.y);
  ui_->CollisionOriginZSpinBox->setValue(cvm_->origin().position.z);

  double r, p, y;
  cvm_->origin().rotation.getRPY(r, p, y);
  ui_->CollisionOriginRollSpinBox->setValue(r);
  ui_->CollisionOriginPitchSpinBox->setValue(p);
  ui_->CollisionOriginYawSpinBox->setValue(y);

  auto gvm = cvm_->geometry();
  ui_->CollisionGeometryBoxLengthSpinBox->setValue(gvm->length());
  ui_->CollisionGeometryBoxWidthSpinBox->setValue(gvm->width());
  ui_->CollisionGeometryBoxHeightSpinBox->setValue(gvm->height());
  ui_->CollisionGeometryCylinderLengthSpinBox->setValue(gvm->length());
  ui_->CollisionGeometryCylinderRadiusSpinBox->setValue(gvm->radius());
  ui_->CollisionGeometrySphereRadiusSpinBox->setValue(gvm->radius());
  ui_->CollisionGeometryMeshPathLineEdit->setText(gvm->filePath());
  ui_->CollisionGeometryMeshScaleSpinBox->setValue(gvm->scale().x);

  arrangeVisualGeometryTypeFrames(
    frame_map_.collision_geom, ui_->CollisionGeometryTypeComboBox->currentText());

  blockSignals(false);
}

void UpdateLinkDialog::readFromVM(const view_model::InertialViewModelPtr& inertial)
{
  blockSignals(true);

  const auto& origin = inertial->origin();
  ui_->InertialOriginXSpinBox->setValue(origin.position.x);
  ui_->InertialOriginYSpinBox->setValue(origin.position.y);
  ui_->InertialOriginZSpinBox->setValue(origin.position.z);
  double r, p, y;
  origin.rotation.getRPY(r, p, y);
  ui_->InertialOriginRollSpinBox->setValue(r);
  ui_->InertialOriginPitchSpinBox->setValue(p);
  ui_->InertialOriginYawSpinBox->setValue(y);

  ui_->InertialMassSpinBox->setValue(vm_->inertial()->mass());

  const auto& inertia = inertial->inertia();
  ui_->InertiaIXXSpinBox->setValue(inertia.ixx);
  ui_->InertiaIXYSpinBox->setValue(inertia.ixy);
  ui_->InertiaIXZSpinBox->setValue(inertia.ixz);
  ui_->InertiaIYYSpinBox->setValue(inertia.iyy);
  ui_->InertiaIYZSpinBox->setValue(inertia.iyz);
  ui_->InertiaIZZSpinBox->setValue(inertia.izz);

  blockSignals(false);
}

void UpdateLinkDialog::readFromUI(const view_model::VisualViewModelPtr& visual)
{
  assert(visual != nullptr);

  urdf::Pose pose;
  pose.position.x = ui_->VisualOriginXSpinBox->value();
  pose.position.y = ui_->VisualOriginYSpinBox->value();
  pose.position.z = ui_->VisualOriginZSpinBox->value();
  pose.rotation.setFromRPY(
    ui_->VisualOriginRollSpinBox->value(), ui_->VisualOriginPitchSpinBox->value(),
    ui_->VisualOriginYawSpinBox->value());
  visual->origin(pose);

  const auto& gvm = visual->geometry();
  switch (gvm->type())
  {
    case GeometryType::BOX:
      gvm->length(ui_->VisualGeometryBoxLengthSpinBox->value());
      gvm->width(ui_->VisualGeometryBoxWidthSpinBox->value());
      gvm->height(ui_->VisualGeometryBoxHeightSpinBox->value());
      break;
    case GeometryType::SPHERE:
      gvm->radius(ui_->VisualGeometrySphereRadiusSpinBox->value());
      break;
    case GeometryType::CYLINDER:
      gvm->radius(ui_->VisualGeometryCylinderRadiusSpinBox->value());
      gvm->length(ui_->VisualGeometryCylinderLengthSpinBox->value());
      break;
    case GeometryType::MESH:
      gvm->filePath(ui_->VisualGeometryMeshPathLineEdit->text());
      const auto scale = ui_->VisualGeometryMeshScaleSpinBox->value();
      gvm->scale(urdf::Vector3(scale, scale, scale));
      break;
  }

  const auto& mvm = vvm_->material();
  mvm->name(ui_->MaterialNameLineEdit->text());
  mvm->color(
    ui_->MaterialColorRedSpinBox->value(), ui_->MaterialColorGreenSpinBox->value(),
    ui_->MaterialColorBlueSpinBox->value());
  mvm->textureFileName(ui_->MaterialTexturePathLineEdit->text());
  vm_->sync();
}

void UpdateLinkDialog::readFromUI(const view_model::CollisionViewModelPtr& collision)
{
  assert(collision != nullptr);

  urdf::Pose pose;
  pose.position.x = ui_->CollisionOriginXSpinBox->value();
  pose.position.y = ui_->CollisionOriginYSpinBox->value();
  pose.position.z = ui_->CollisionOriginZSpinBox->value();
  pose.rotation.setFromRPY(
    ui_->CollisionOriginRollSpinBox->value(), ui_->CollisionOriginPitchSpinBox->value(),
    ui_->CollisionOriginYawSpinBox->value());
  collision->origin(pose);

  const auto& gvm = collision->geometry();
  switch (gvm->type())
  {
    case GeometryType::BOX:
      gvm->length(ui_->CollisionGeometryBoxLengthSpinBox->value());
      gvm->width(ui_->CollisionGeometryBoxWidthSpinBox->value());
      gvm->height(ui_->CollisionGeometryBoxHeightSpinBox->value());
      break;
    case GeometryType::SPHERE:
      gvm->radius(ui_->CollisionGeometrySphereRadiusSpinBox->value());
      break;
    case GeometryType::CYLINDER:
      gvm->radius(ui_->CollisionGeometryCylinderRadiusSpinBox->value());
      gvm->length(ui_->CollisionGeometryCylinderLengthSpinBox->value());
      break;
    case GeometryType::MESH:
      gvm->filePath(ui_->CollisionGeometryMeshPathLineEdit->text());
      const auto scale = ui_->CollisionGeometryMeshScaleSpinBox->value();
      gvm->scale(urdf::Vector3(scale, scale, scale));
      break;
  }
  vm_->sync();
}

void UpdateLinkDialog::readFromUI(const view_model::JointViewModelPtr& joint)
{
  joint->name(ui_->JointNameLineEdit->text());
  joint->parentLinkName(ui_->JointParentLinkComboBox->currentText());
  joint->childLinkName(ui_->LinkNameLineEdit->text());
  joint->type(ui_->JointTypeComboBox->currentText());

  if (joint->limitsEnabled())
  {
    joint->limits()->lower(ui_->JointLimitLowerSpinBox->value());
    joint->limits()->upper(ui_->JointLimitUpperSpinBox->value());
    joint->limits()->effort(ui_->JointLimitEffortSpinBox->value());
    joint->limits()->velocity(ui_->JointLimitVelocitySpinBox->value());
  }

  urdf::Pose pose;
  pose.position.x = ui_->JointOriginXSpinBox->value();
  pose.position.y = ui_->JointOriginYSpinBox->value();
  pose.position.z = ui_->JointOriginZSpinBox->value();
  pose.rotation.setFromRPY(
    ui_->JointOriginRollSpinBox->value(), ui_->JointOriginPitchSpinBox->value(),
    ui_->JointOriginYawSpinBox->value());
  joint->origin(pose);

  urdf::Vector3 axis;
  axis.x = ui_->JointAxisXSpinBox->value();
  axis.y = ui_->JointAxisYSpinBox->value();
  axis.z = ui_->JointAxisZSpinBox->value();
  joint->axis(axis);

  vm_->sync();
}

void UpdateLinkDialog::readFromUI(const view_model::InertialViewModelPtr& inertial)
{
  urdf::Pose pose;
  pose.position.x = ui_->InertialOriginXSpinBox->value();
  pose.position.y = ui_->InertialOriginYSpinBox->value();
  pose.position.z = ui_->InertialOriginZSpinBox->value();
  pose.rotation.setFromRPY(
    ui_->InertialOriginRollSpinBox->value(), ui_->InertialOriginPitchSpinBox->value(),
    ui_->InertialOriginYawSpinBox->value());
  inertial->origin(pose);

  inertial->mass(ui_->InertialMassSpinBox->value());

  view_model::Inertia inertia{};
  inertia.ixx = ui_->InertiaIXXSpinBox->value();
  inertia.ixy = ui_->InertiaIXYSpinBox->value();
  inertia.ixz = ui_->InertiaIXZSpinBox->value();
  inertia.iyy = ui_->InertiaIYYSpinBox->value();
  inertia.iyz = ui_->InertiaIYZSpinBox->value();
  inertia.izz = ui_->InertiaIZZSpinBox->value();
  inertial->inertia(inertia);

  vm_->sync();
}

void UpdateLinkDialog::blockSignals(bool block)
{
  const QList<QWidget*> widget_list = this->findChildren<QWidget*>();
  const QList<QWidget*>::const_iterator last_widget(widget_list.end());
  QList<QWidget*>::const_iterator widget_iter(widget_list.begin());

  while (widget_iter != last_widget)
  {
    (*widget_iter)->blockSignals(block);
    ++widget_iter;
  }
}

void UpdateLinkDialog::emitChanged()
{
  Q_EMIT Changed();
}

void UpdateLinkDialog::arrangeVisualGeometryTypeFrames(
  const map<QString, QFrame*>& map,
  const QString& frameName)
{
  for (const auto& pair : map)
    pair.second->hide();
  map.at(frameName)->show();
}
}  // namespace ui
}  // namespace urdf_builder
