#include <filesystem>
#include <boost/polymorphic_cast.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/env.h>

#include <tobas_std_tools/console.hpp>
#include <tobas_constants/constants.hpp>

#include "../../include/tobas_urdf_builder_plugin/ui/urdf_builder_panel.hpp"
#include "../../include/tobas_urdf_builder_plugin/ui/update_link_dialog.hpp"
#include "../../include/tobas_urdf_builder_plugin/ui/double_map_input_dialog.hpp"
#include "../../include/tobas_urdf_builder_plugin/ui/string_input_dialog.hpp"
#include "../../include/tobas_urdf_builder_plugin/ui/widget_item.hpp"
#include "../../include/tobas_urdf_builder_plugin/utils/constants.hpp"
#include "ui_update_link_dialog.h"

using namespace std;

namespace gui
{
namespace urdf_builder
{
namespace ui
{
UpdateLinkDialog::UpdateLinkDialog(rclcpp::Node::SharedPtr node, URDFBuilderPanel* main)
  : QDialog(main),
    node_(node),
    property_client_(node, tobas::kPropertyServerName, kPropertySection),
    main_(main),
    ui_(new Ui::UpdateLinkDialogUI()),
    link_vm_(new view_model::LinkViewModel())
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

  readFromVM(link_vm_);
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

void UpdateLinkDialog::readFromVM(const view_model::LinkViewModelPtr& link_vm)
{
  link_vm_ = link_vm->clone();

  blockSignals(true);

  ui_->VisualListWidget->clear();
  for (const auto& visual : link_vm_->visuals())
    ui_->VisualListWidget->addItem(new VisualListWidgetItem(visual));

  ui_->CollisionListWidget->clear();
  for (const auto& collision : link_vm_->collisions())
    ui_->CollisionListWidget->addItem(new CollisionListWidgetItem(collision));

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
  return link_vm_;
}

void UpdateLinkDialog::VisualGeometryTypeComboBoxIndexChanged(int)
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::VisualGeometryTypeComboBoxIndexChanged");

  const auto geometry_type = ui_->VisualGeometryTypeComboBox->currentText();
  arrangeVisualGeometryTypeFrames(frame_map_.visual_geom, geometry_type);
  visual_vm_->geometry()->type(geometry_type);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::CollisionGeometryTypeComboBoxIndexChanged(int)
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::CollisionGeometryTypeComboBoxIndexChange");

  const auto geometry_type = ui_->CollisionGeometryTypeComboBox->currentText();
  arrangeVisualGeometryTypeFrames(frame_map_.collision_geom, geometry_type);
  collision_vm_->geometry()->type(geometry_type);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::LinkNameLineEditTextChanged(const QString& text)
{
  RCLCPP_DEBUG_STREAM(
    node_->get_logger(), "UpdateLinkDialog::LinkNameLineEditTextChanged(" << text.toStdString() << ")");

  link_vm_->name(text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::JointNameLineEditTextChanged(const QString& text)
{
  RCLCPP_DEBUG_STREAM(
    node_->get_logger(), "UpdateLinkDialog::JointNameLineEditTextChanged(" << text.toStdString() << ")");

  link_vm_->joint()->name(text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::VisualNameLineEditTextChanged(const QString& text)
{
  RCLCPP_DEBUG_STREAM(
    node_->get_logger(), "UpdateLinkDialog::VisualNameLineEditTextChanged(" << text.toStdString() << ")");

  visual_vm_->name(text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::VisualGeometryMeshPathLineEditTextChanged(const QString& text)
{
  RCLCPP_DEBUG_STREAM(
    node_->get_logger(), "UpdateLinkDialog::VisualGeometryMeshPathLineEditTextChanged(" << text.toStdString() << ")");

  visual_vm_->geometry()->filePath(text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::CollisionNameLineEditTextChanged(const QString& text)
{
  RCLCPP_DEBUG_STREAM(
    node_->get_logger(), "UpdateLinkDialog::CollisionNameLineEditTextChanged(" << text.toStdString() << ")");

  collision_vm_->name(text);

  emitChanged();
}

void UpdateLinkDialog::CollisionGeometryMeshPathEditTextChanged(const QString& text)
{
  RCLCPP_DEBUG_STREAM(
    node_->get_logger(), "UpdateLinkDialog::CollisionGeometryMeshPathEditTextChanged(" << text.toStdString() << ")");

  collision_vm_->geometry()->filePath(text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::MaterialNameLineEditTextChanged(const QString& text)
{
  RCLCPP_DEBUG_STREAM(
    node_->get_logger(), "UpdateLinkDialog::MaterialNameLineEditTextChanged(" << text.toStdString() << ")");

  visual_vm_->material()->name(text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::MaterialTexturePathLineEditTextChanged(const QString& text)
{
  RCLCPP_DEBUG_STREAM(
    node_->get_logger(), "UpdateLinkDialog::MaterialTexturePathLineEditTextChanged(" << text.toStdString() << ")");

  visual_vm_->material()->textureFileName(text);
  link_vm_->sync();

  emitChanged();
}

void UpdateLinkDialog::JointParentComboBoxIndexChanged(int)
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::JointParentComboBoxIndexChanged");

  readFromUI(link_vm_->joint());

  emitChanged();
}

void UpdateLinkDialog::JointTypeComboBoxIndexChanged(int)
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::JointTypeComboBoxIndexChanged");

  readFromUI(link_vm_->joint());

  const auto& joint = link_vm_->joint();
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
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::JointSpinBoxValueChanged");

  readFromUI(link_vm_->joint());
  emitChanged();
}

void UpdateLinkDialog::VisualSpinBoxValueChanged(double)
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::VisualSpinBoxValueChanged");

  readFromUI(visual_vm_);
  emitChanged();
}

void UpdateLinkDialog::CollisionSpinBoxValueChanged(double)
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::CollisionSpinBoxValueChanged");

  readFromUI(collision_vm_);
  emitChanged();
}

void UpdateLinkDialog::InertialSpinBoxValueChanged(double)
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::InertialSpinBoxValueChanged");

  readFromUI(link_vm_->inertial());
  emitChanged();
}

void UpdateLinkDialog::VisualListWidgetItemClicked(QListWidgetItem* item)
{
  RCLCPP_DEBUG_STREAM(
    node_->get_logger(), "UpdateLinkDialog::VisualListWidgetItemClicked(" << item->text().toStdString() << ")");

  const auto visual_item = boost::polymorphic_downcast<VisualListWidgetItem*>(item);
  readFromVM(visual_item->viewModel());
}

void UpdateLinkDialog::CollisionListWidgetItemClicked(QListWidgetItem* item)
{
  RCLCPP_DEBUG_STREAM(
    node_->get_logger(), "UpdateLinkDialog::CollisionListWidgetItemClicked(" << item->text().toStdString() << ")");

  const auto collision_item = boost::polymorphic_downcast<CollisionListWidgetItem*>(item);
  readFromVM(collision_item->viewModel());
}

void UpdateLinkDialog::RenameLinkButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::RenameLinkButtonClicked");

  const auto cur_name = ui_->LinkNameLineEdit->text();
  auto excludeds = main_->linkNames();
  excludeds.removeOne(cur_name);
  StringInputDialog dialog(this, "Rename Link", "Link Name", cur_name, excludeds);

  const auto result = dialog.exec();
  if (result != QDialog::Accepted)
    return;

  ui_->LinkNameLineEdit->setText(dialog.getText());

  // FIXME: LinkNameLineEditの変更時とここで2回リロードしないとTreeNodeが重複する
  emitChanged();
}

void UpdateLinkDialog::RenameJointButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::RenameJointButtonClicked");

  const auto cur_name = ui_->JointNameLineEdit->text();
  auto excludeds = main_->jointNames();
  excludeds.removeOne(cur_name);
  StringInputDialog dialog(this, "Rename Joint", "Joint Name", cur_name, excludeds);

  const auto result = dialog.exec();
  if (result != QDialog::Accepted)
    return;

  ui_->JointNameLineEdit->setText(dialog.getText());

  emitChanged();
}

void UpdateLinkDialog::AddVisualButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::AddVisualButtonClicked");

  const auto visual_vm = make_shared<view_model::VisualViewModel>(nullptr);
  const auto item = new VisualListWidgetItem(visual_vm);
  ui_->VisualListWidget->addItem(item);
  ui_->VisualListWidget->setCurrentItem(item);
  link_vm_->add(visual_vm);
  readFromVM(visual_vm);

  emitChanged();
}

void UpdateLinkDialog::RemoveVisualButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::RemoveVisualButtonClicked");

  if (ui_->VisualListWidget->selectedItems().empty())
  {
    QMessageBox::warning(this, kError, "No visual is selected.");
    return;
  }

  const auto item = ui_->VisualListWidget->selectedItems().front();
  const auto casted_item = boost::polymorphic_downcast<VisualListWidgetItem*>(item);
  link_vm_->remove(casted_item->viewModel());
  ui_->VisualListWidget->removeItemWidget(item);
  delete item;

  if (ui_->VisualListWidget->count() > 0)
  {
    const auto first = boost::polymorphic_downcast<VisualListWidgetItem*>(ui_->VisualListWidget->item(0));
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
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::AddCollisionButtonClicked");

  const auto collision_vm = make_shared<view_model::CollisionViewModel>(nullptr);
  const auto item = new CollisionListWidgetItem(collision_vm);
  ui_->CollisionListWidget->addItem(item);
  ui_->CollisionListWidget->setCurrentItem(item);
  link_vm_->add(collision_vm);
  readFromVM(collision_vm);

  emitChanged();
}

void UpdateLinkDialog::RemoveCollisionButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::RemoveCollisionButtonClicked");

  if (ui_->CollisionListWidget->selectedItems().empty())
  {
    QMessageBox::warning(this, kError, "No collision is selected.");
    return;
  }

  const auto item = ui_->CollisionListWidget->selectedItems().front();
  const auto casted_item = boost::polymorphic_downcast<CollisionListWidgetItem*>(item);
  link_vm_->remove(casted_item->viewModel());
  ui_->CollisionListWidget->removeItemWidget(item);
  delete item;

  if (ui_->CollisionListWidget->count() > 0)
  {
    const auto first = boost::polymorphic_downcast<CollisionListWidgetItem*>(ui_->CollisionListWidget->item(0));
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
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::VisualGeometryMeshBrowseButtonClicked");

  // 最後に開いたディレクトリを取得
  string last_dir;
  if (property_client_.get(kConfigKey_VisualGeometryMeshBrowseDir, last_dir) < 0)
  {
    PRINT_WARN(property_client_.errorMessage());
    last_dir = rcutils_get_home_dir();
  }

  // メッシュファイルのパスを取得
  const auto file_path = QFileDialog::getOpenFileName(
    this, tr("URDF Builder"), QString::fromStdString(last_dir), tr("Mesh Files (*.stl *.dae);;All Files (*)"));
  if (file_path.isEmpty())
    return;

  // メッシュファイルのパスを設定
  ui_->VisualGeometryMeshPathLineEdit->setText("file://" + file_path);

  // 最後に開いたディレクトリを保存
  const auto new_dir = filesystem::path(file_path.toStdString()).parent_path().string();
  if (property_client_.set(kConfigKey_VisualGeometryMeshBrowseDir, new_dir) < 0)
  {
    PRINT_WARN(property_client_.errorMessage());
    return;
  }
  if (property_client_.save() < 0)
  {
    PRINT_WARN(property_client_.errorMessage());
    return;
  }
}

void UpdateLinkDialog::CollisionGeometryMeshBrowseButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::CollisionGeometryMeshBrowseButtonClicked");

  // 最後に開いたディレクトリを取得
  string last_dir;
  if (property_client_.get(kConfigKey_CollisionGeometryMeshBrowseDir, last_dir) < 0)
  {
    PRINT_WARN(property_client_.errorMessage());
    last_dir = rcutils_get_home_dir();
  }

  // メッシュファイルのパスを取得
  const auto file_path = QFileDialog::getOpenFileName(
    this, tr("URDF Builder"), QString::fromStdString(last_dir), tr("Mesh Files (*.stl *.dae);;All Files (*)"));
  if (file_path.isEmpty())
    return;

  // メッシュファイルのパスを設定
  ui_->CollisionGeometryMeshPathLineEdit->setText("file://" + file_path);

  // 最後に開いたディレクトリを保存
  const auto new_dir = filesystem::path(file_path.toStdString()).parent_path().string();
  if (property_client_.set(kConfigKey_CollisionGeometryMeshBrowseDir, new_dir) < 0)
  {
    PRINT_WARN(property_client_.errorMessage());
    return;
  }
  if (property_client_.save() < 0)
  {
    PRINT_WARN(property_client_.errorMessage());
    return;
  }
}

void UpdateLinkDialog::MaterialColorPickButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::MaterialColorPickButtonClicked");

  const auto& color = visual_vm_->material()->color();
  QColorDialog dialog(QColor::fromRgbF(color.r, color.g, color.b, color.a));

  if (dialog.exec() != QDialog::Accepted)
    return;

  visual_vm_->material()->color(dialog.currentColor());
  link_vm_->sync();
  readFromVM(visual_vm_);

  emitChanged();
}

void UpdateLinkDialog::BuildInertiaBoxButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::BuildInertiaBoxButtonClicked");

  DoubleMapInputDialog dialog(this, "Box Inertia", { "X", "Y", "Z" });
  const auto result = dialog.exec();

  if (result != QDialog::Accepted)
    return;

  const auto& x = dialog.getValue("X");
  const auto& y = dialog.getValue("Y");
  const auto& z = dialog.getValue("Z");

  link_vm_->inertial()->buildInertiaBox(x, y, z);
  link_vm_->sync();
  readFromVM(link_vm_->inertial());

  emitChanged();
}

void UpdateLinkDialog::BuildInertiaCylinderButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::BuildInertiaCylinderButtonClicked");

  DoubleMapInputDialog dialog(this, "Cylinder Inertia", { "Radius", "Length" });
  const auto result = dialog.exec();

  if (result != QDialog::Accepted)
    return;

  const auto& radius = dialog.getValue("Radius");
  const auto& length = dialog.getValue("Length");

  link_vm_->inertial()->buildInertiaCylinder(radius, length);
  link_vm_->sync();
  readFromVM(link_vm_->inertial());

  emitChanged();
}

void UpdateLinkDialog::BuildInertiaSphereButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "UpdateLinkDialog::BuildInertiaSphereButtonClicked");

  DoubleMapInputDialog dialog(this, "Sphere Inertia", { "Radius" });
  const auto result = dialog.exec();

  if (result != QDialog::Accepted)
    return;

  const auto& radius = dialog.getValue("Radius");

  link_vm_->inertial()->buildInertiaSphere(radius);
  link_vm_->sync();
  readFromVM(link_vm_->inertial());

  emitChanged();
}

void UpdateLinkDialog::readFromVM(const view_model::JointViewModelPtr& joint)
{
  blockSignals(true);

  ui_->JointNameLineEdit->setText(joint->name());

  ui_->JointParentLinkComboBox->clear();
  for (const auto& link_name : main_->linkNames())
    if (link_name != link_vm_->name())
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
  visual_vm_ = visual;
  ui_->VisualOriginGroupBox->setVisible(visual != nullptr);
  ui_->VisualGeometryGroupBox->setVisible(visual != nullptr);
  ui_->VisualMaterialGroupBox->setVisible(visual != nullptr);

  if (!visual)
    return;

  blockSignals(true);

  ui_->VisualNameLineEdit->setText(visual->name());
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

void UpdateLinkDialog::readFromVM(const view_model::CollisionViewModelPtr& collision)
{
  collision_vm_ = collision;
  ui_->CollisionOriginGroupBox->setVisible(collision != nullptr);
  ui_->CollisionGeometryGroupBox->setVisible(collision != nullptr);

  if (!collision)
    return;

  blockSignals(true);

  ui_->CollisionNameLineEdit->setText(collision->name());
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

  ui_->InertialMassSpinBox->setValue(link_vm_->inertial()->mass());

  const auto& inertia = inertial->inertia();
  ui_->InertiaIXXSpinBox->setValue(inertia.ixx);
  ui_->InertiaIXYSpinBox->setValue(inertia.ixy);
  ui_->InertiaIXZSpinBox->setValue(inertia.ixz);
  ui_->InertiaIYYSpinBox->setValue(inertia.iyy);
  ui_->InertiaIYZSpinBox->setValue(inertia.iyz);
  ui_->InertiaIZZSpinBox->setValue(inertia.izz);

  blockSignals(false);
}

void UpdateLinkDialog::readFromUI(const view_model::VisualViewModelPtr& visual) const
{
  assert(visual);

  urdf::Pose pose;
  pose.position.x = ui_->VisualOriginXSpinBox->value();
  pose.position.y = ui_->VisualOriginYSpinBox->value();
  pose.position.z = ui_->VisualOriginZSpinBox->value();
  pose.rotation.setFromRPY(
    ui_->VisualOriginRollSpinBox->value(), ui_->VisualOriginPitchSpinBox->value(),
    ui_->VisualOriginYawSpinBox->value());
  visual->origin(pose);

  const auto& geometry_vm = visual->geometry();
  switch (geometry_vm->type())
  {
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
      geometry_vm->scale(urdf::Vector3(scale, scale, scale));
      break;
  }

  const auto& material_vm = visual_vm_->material();
  material_vm->name(ui_->MaterialNameLineEdit->text());
  material_vm->color(
    ui_->MaterialColorRedSpinBox->value(), ui_->MaterialColorGreenSpinBox->value(),
    ui_->MaterialColorBlueSpinBox->value());
  material_vm->textureFileName(ui_->MaterialTexturePathLineEdit->text());
  link_vm_->sync();
}

void UpdateLinkDialog::readFromUI(const view_model::CollisionViewModelPtr& collision) const
{
  assert(collision);

  urdf::Pose pose;
  pose.position.x = ui_->CollisionOriginXSpinBox->value();
  pose.position.y = ui_->CollisionOriginYSpinBox->value();
  pose.position.z = ui_->CollisionOriginZSpinBox->value();
  pose.rotation.setFromRPY(
    ui_->CollisionOriginRollSpinBox->value(), ui_->CollisionOriginPitchSpinBox->value(),
    ui_->CollisionOriginYawSpinBox->value());
  collision->origin(pose);

  const auto& geometry_vm = collision->geometry();
  switch (geometry_vm->type())
  {
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
      geometry_vm->scale(urdf::Vector3(scale, scale, scale));
      break;
  }
  link_vm_->sync();
}

void UpdateLinkDialog::readFromUI(const view_model::JointViewModelPtr& joint) const
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
    ui_->JointOriginRollSpinBox->value(), ui_->JointOriginPitchSpinBox->value(), ui_->JointOriginYawSpinBox->value());
  joint->origin(pose);

  urdf::Vector3 axis;
  axis.x = ui_->JointAxisXSpinBox->value();
  axis.y = ui_->JointAxisYSpinBox->value();
  axis.z = ui_->JointAxisZSpinBox->value();
  joint->axis(axis);

  link_vm_->sync();
}

void UpdateLinkDialog::readFromUI(const view_model::InertialViewModelPtr& inertial) const
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

  view_model::Inertia inertia;
  inertia.ixx = ui_->InertiaIXXSpinBox->value();
  inertia.ixy = ui_->InertiaIXYSpinBox->value();
  inertia.ixz = ui_->InertiaIXZSpinBox->value();
  inertia.iyy = ui_->InertiaIYYSpinBox->value();
  inertia.iyz = ui_->InertiaIYZSpinBox->value();
  inertia.izz = ui_->InertiaIZZSpinBox->value();
  inertial->inertia(inertia);

  link_vm_->sync();
}

void UpdateLinkDialog::blockSignals(bool block)
{
  const auto widget_list = findChildren<QWidget*>();
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

void UpdateLinkDialog::arrangeVisualGeometryTypeFrames(const map<QString, QFrame*>& map, const QString& type)
{
  for (const auto& pair : map)
    pair.second->hide();
  map.at(type)->show();
}
}  // namespace ui
}  // namespace urdf_builder
}  // namespace gui
