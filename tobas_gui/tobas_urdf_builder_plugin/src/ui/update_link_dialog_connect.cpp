#include "tobas_urdf_builder_plugin/ui/update_link_dialog.hpp"

#include "ui_update_link_dialog.h"

namespace gui
{
namespace ub
{
namespace ui
{
void UpdateLinkDialog::defineConnections()
{
  connect(ui_->LinkNameLineEdit, &QLineEdit::textChanged, this, &self::onLinkNameLineEditTextChanged);
  connect(ui_->JointNameLineEdit, &QLineEdit::textChanged, this, &self::onJointNameLineEditTextChanged);
  connect(ui_->VisualNameLineEdit, &QLineEdit::textChanged, this, &self::onVisualNameLineEditTextChanged);
  connect(
    ui_->VisualGeometryMeshPathLineEdit,
    &QLineEdit::textChanged,
    this,
    &self::onVisualGeometryMeshPathLineEditTextChanged);

  connect(ui_->CollisionNameLineEdit, &QLineEdit::textChanged, this, &self::onCollisionNameLineEditTextChanged);
  connect(
    ui_->CollisionGeometryMeshPathLineEdit,
    &QLineEdit::textChanged,
    this,
    &self::onCollisionGeometryMeshPathEditTextChanged);

  connect(ui_->MaterialNameLineEdit, &QLineEdit::textChanged, this, &self::onMaterialNameLineEditTextChanged);
  connect(
    ui_->MaterialTexturePathLineEdit, &QLineEdit::textChanged, this, &self::onMaterialTexturePathLineEditTextChanged);

  connect(
    ui_->VisualGeometryTypeComboBox,
    QOverload<int>::of(&QComboBox::currentIndexChanged),
    this,
    &self::onVisualGeometryTypeComboBoxIndexChanged);
  connect(
    ui_->CollisionGeometryTypeComboBox,
    QOverload<int>::of(&QComboBox::currentIndexChanged),
    this,
    &self::onCollisionGeometryTypeComboBoxIndexChanged);

  connect(
    ui_->VisualOriginXSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginYSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginZSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginRollSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginPitchSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginYawSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryBoxWidthSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryBoxLengthSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryBoxHeightSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometrySphereRadiusSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryCylinderLengthSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryCylinderRadiusSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryMeshScaleSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->MaterialColorRedSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->MaterialColorGreenSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->MaterialColorBlueSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);

  connect(
    ui_->CollisionOriginXSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginYSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginZSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginRollSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginPitchSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginYawSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);

  connect(
    ui_->CollisionGeometryBoxWidthSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryBoxLengthSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryBoxHeightSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometrySphereRadiusSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryCylinderLengthSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryCylinderRadiusSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryMeshScaleSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);

  connect(ui_->VisualListWidget, &QListWidget::itemClicked, this, &self::onVisualListWidgetItemClicked);
  connect(ui_->CollisionListWidget, &QListWidget::itemClicked, this, &self::onCollisionListWidgetItemClicked);

  connect(
    ui_->JointOriginXSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointOriginYSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointOriginZSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointOriginRollSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointOriginPitchSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointOriginYawSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointAxisXSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointAxisYSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointAxisZSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointLimitLowerSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointLimitUpperSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointLimitEffortSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointLimitVelocitySpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);

  connect(
    ui_->JointParentLinkComboBox,
    QOverload<int>::of(&QComboBox::currentIndexChanged),
    this,
    &self::onJointParentComboBoxIndexChanged);
  connect(
    ui_->JointTypeComboBox,
    QOverload<int>::of(&QComboBox::currentIndexChanged),
    this,
    &self::onJointTypeComboBoxIndexChanged);

  connect(
    ui_->InertialOriginXSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginYSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginZSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginRollSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginPitchSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginYawSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertialMassSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIXXSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIXYSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIXZSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIYYSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIYZSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIZZSpinBox,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);

  connect(ui_->RenameLinkButton, &QPushButton::released, this, &self::onRenameLinkButtonClicked);
  connect(ui_->RenameJointButton, &QPushButton::released, this, &self::onRenameJointButtonClicked);
  connect(ui_->AddVisualButton, &QPushButton::released, this, &self::onAddVisualButtonClicked);
  connect(ui_->RemoveVisualButton, &QPushButton::released, this, &self::onRemoveVisualButtonClicked);
  connect(ui_->AddCollisionButton, &QPushButton::released, this, &self::onAddCollisionButtonClicked);
  connect(ui_->RemoveCollisionButton, &QPushButton::released, this, &self::onRemoveCollisionButtonClicked);
  connect(
    ui_->VisualGeometryMeshBrowseButton, &QPushButton::released, this, &self::onVisualGeometryMeshBrowseButtonClicked);
  connect(
    ui_->CollisionGeometryMeshBrowseButton,
    &QPushButton::released,
    this,
    &self::onCollisionGeometryMeshBrowseButtonClicked);
  connect(ui_->MaterialColorPickButton, &QPushButton::released, this, &self::onMaterialColorPickButtonClicked);
  connect(ui_->BuildInertiaBoxButton, &QPushButton::released, this, &self::onBuildInertiaBoxButtonClicked);
  connect(ui_->BuildInertiaCylinderButton, &QPushButton::released, this, &self::onBuildInertiaCylinderButtonClicked);
  connect(ui_->BuildInertiaSphereButton, &QPushButton::released, this, &self::onBuildInertiaSphereButtonClicked);
}
}  // namespace ui
}  // namespace ub
}  // namespace gui
