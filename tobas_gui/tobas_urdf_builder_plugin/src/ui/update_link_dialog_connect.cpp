#include "../../include/tobas_urdf_builder_plugin/ui/update_link_dialog.hpp"
#include "ui_update_link_dialog.h"

namespace gui
{
namespace urdf_builder
{
namespace ui
{
void UpdateLinkDialog::defineConnections()
{
  connect(ui_->LinkNameLineEdit, &QLineEdit::textChanged, this, &self::LinkNameLineEditTextChanged);
  connect(ui_->JointNameLineEdit, &QLineEdit::textChanged, this, &self::JointNameLineEditTextChanged);
  connect(ui_->VisualNameLineEdit, &QLineEdit::textChanged, this, &self::VisualNameLineEditTextChanged);
  connect(
    ui_->VisualGeometryMeshPathLineEdit, &QLineEdit::textChanged, this,
    &self::VisualGeometryMeshPathLineEditTextChanged);

  connect(ui_->CollisionNameLineEdit, &QLineEdit::textChanged, this, &self::CollisionNameLineEditTextChanged);
  connect(
    ui_->CollisionGeometryMeshPathLineEdit, &QLineEdit::textChanged, this,
    &self::CollisionGeometryMeshPathEditTextChanged);

  connect(ui_->MaterialNameLineEdit, &QLineEdit::textChanged, this, &self::MaterialNameLineEditTextChanged);
  connect(
    ui_->MaterialTexturePathLineEdit, &QLineEdit::textChanged, this, &self::MaterialTexturePathLineEditTextChanged);

  connect(
    ui_->VisualGeometryTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    &self::VisualGeometryTypeComboBoxIndexChanged);
  connect(
    ui_->CollisionGeometryTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    &self::CollisionGeometryTypeComboBoxIndexChanged);

  connect(
    ui_->VisualOriginXSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginYSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginZSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginRollSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginPitchSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginYawSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryBoxWidthSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryBoxLengthSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryBoxHeightSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometrySphereRadiusSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryCylinderLengthSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryCylinderRadiusSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryMeshScaleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->MaterialColorRedSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->MaterialColorGreenSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);
  connect(
    ui_->MaterialColorBlueSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::VisualSpinBoxValueChanged);

  connect(
    ui_->CollisionOriginXSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::CollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginYSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::CollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginZSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::CollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginRollSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::CollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginPitchSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::CollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginYawSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::CollisionSpinBoxValueChanged);

  connect(
    ui_->CollisionGeometryBoxWidthSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::CollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryBoxLengthSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::CollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryBoxHeightSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::CollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometrySphereRadiusSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::CollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryCylinderLengthSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::CollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryCylinderRadiusSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::CollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryMeshScaleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::CollisionSpinBoxValueChanged);

  connect(ui_->VisualListWidget, &QListWidget::itemClicked, this, &self::VisualListWidgetItemClicked);
  connect(ui_->CollisionListWidget, &QListWidget::itemClicked, this, &self::CollisionListWidgetItemClicked);

  connect(
    ui_->JointOriginXSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::JointSpinBoxValueChanged);
  connect(
    ui_->JointOriginYSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::JointSpinBoxValueChanged);
  connect(
    ui_->JointOriginZSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::JointSpinBoxValueChanged);
  connect(
    ui_->JointOriginRollSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::JointSpinBoxValueChanged);
  connect(
    ui_->JointOriginPitchSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::JointSpinBoxValueChanged);
  connect(
    ui_->JointOriginYawSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::JointSpinBoxValueChanged);
  connect(
    ui_->JointAxisXSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &self::JointSpinBoxValueChanged);
  connect(
    ui_->JointAxisYSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &self::JointSpinBoxValueChanged);
  connect(
    ui_->JointAxisZSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &self::JointSpinBoxValueChanged);
  connect(
    ui_->JointLimitLowerSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::JointSpinBoxValueChanged);
  connect(
    ui_->JointLimitUpperSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::JointSpinBoxValueChanged);
  connect(
    ui_->JointLimitEffortSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::JointSpinBoxValueChanged);
  connect(
    ui_->JointLimitVelocitySpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::JointSpinBoxValueChanged);

  connect(
    ui_->JointParentLinkComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    &self::JointParentComboBoxIndexChanged);
  connect(
    ui_->JointTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    &self::JointTypeComboBoxIndexChanged);

  connect(
    ui_->InertialOriginXSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::InertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginYSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::InertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginZSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::InertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginRollSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::InertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginPitchSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::InertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginYawSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::InertialSpinBoxValueChanged);
  connect(
    ui_->InertialMassSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::InertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIXXSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::InertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIXYSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::InertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIXZSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::InertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIYYSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::InertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIYZSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::InertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIZZSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &self::InertialSpinBoxValueChanged);

  connect(ui_->RenameLinkButton, &QPushButton::released, this, &self::RenameLinkButtonClicked);
  connect(ui_->RenameJointButton, &QPushButton::released, this, &self::RenameJointButtonClicked);
  connect(ui_->AddVisualButton, &QPushButton::released, this, &self::AddVisualButtonClicked);
  connect(ui_->RemoveVisualButton, &QPushButton::released, this, &self::RemoveVisualButtonClicked);
  connect(ui_->AddCollisionButton, &QPushButton::released, this, &self::AddCollisionButtonClicked);
  connect(ui_->RemoveCollisionButton, &QPushButton::released, this, &self::RemoveCollisionButtonClicked);
  connect(
    ui_->VisualGeometryMeshBrowseButton, &QPushButton::released, this, &self::VisualGeometryMeshBrowseButtonClicked);
  connect(
    ui_->CollisionGeometryMeshBrowseButton, &QPushButton::released, this,
    &self::CollisionGeometryMeshBrowseButtonClicked);
  connect(ui_->MaterialColorPickButton, &QPushButton::released, this, &self::MaterialColorPickButtonClicked);
  connect(ui_->BuildInertiaBoxButton, &QPushButton::released, this, &self::BuildInertiaBoxButtonClicked);
  connect(ui_->BuildInertiaCylinderButton, &QPushButton::released, this, &self::BuildInertiaCylinderButtonClicked);
  connect(ui_->BuildInertiaSphereButton, &QPushButton::released, this, &self::BuildInertiaSphereButtonClicked);
}
}  // namespace ui
}  // namespace urdf_builder
}  // namespace gui
