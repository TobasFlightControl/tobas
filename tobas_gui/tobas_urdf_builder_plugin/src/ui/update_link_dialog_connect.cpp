// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/ui/update_link_dialog.hpp"

#include "ui_update_link_dialog.h"

namespace tobas
{
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
    qOverload<int>(&QComboBox::currentIndexChanged),
    this,
    &self::onVisualGeometryTypeComboBoxIndexChanged);
  connect(
    ui_->CollisionGeometryTypeComboBox,
    qOverload<int>(&QComboBox::currentIndexChanged),
    this,
    &self::onCollisionGeometryTypeComboBoxIndexChanged);

  connect(
    ui_->VisualOriginXSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginYSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginZSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginRollSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginPitchSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualOriginYawSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryBoxWidthSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryBoxLengthSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryBoxHeightSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometrySphereRadiusSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryCylinderLengthSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryCylinderRadiusSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->VisualGeometryMeshScaleSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->MaterialColorRedSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->MaterialColorGreenSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);
  connect(
    ui_->MaterialColorBlueSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onVisualSpinBoxValueChanged);

  connect(
    ui_->CollisionOriginXSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginYSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginZSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginRollSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginPitchSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionOriginYawSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);

  connect(
    ui_->CollisionGeometryBoxWidthSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryBoxLengthSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryBoxHeightSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometrySphereRadiusSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryCylinderLengthSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryCylinderRadiusSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);
  connect(
    ui_->CollisionGeometryMeshScaleSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onCollisionSpinBoxValueChanged);

  connect(ui_->VisualListWidget, &QListWidget::itemClicked, this, &self::onVisualListWidgetItemClicked);
  connect(ui_->CollisionListWidget, &QListWidget::itemClicked, this, &self::onCollisionListWidgetItemClicked);

  connect(
    ui_->JointOriginXSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointOriginYSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointOriginZSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointOriginRollSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointOriginPitchSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointOriginYawSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointAxisXSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointAxisYSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointAxisZSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointLimitLowerSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointLimitUpperSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointLimitEffortSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);
  connect(
    ui_->JointLimitVelocitySpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onJointSpinBoxValueChanged);

  connect(
    ui_->JointParentLinkComboBox,
    qOverload<int>(&QComboBox::currentIndexChanged),
    this,
    &self::onJointParentComboBoxIndexChanged);
  connect(
    ui_->JointTypeComboBox,
    qOverload<int>(&QComboBox::currentIndexChanged),
    this,
    &self::onJointTypeComboBoxIndexChanged);

  connect(
    ui_->InertialOriginXSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginYSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginZSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginRollSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginPitchSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertialOriginYawSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertialMassSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIXXSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIXYSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIXZSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIYYSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIYZSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
    this,
    &self::onInertialSpinBoxValueChanged);
  connect(
    ui_->InertiaIZZSpinBox,
    qOverload<double>(&QDoubleSpinBox::valueChanged),
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
}  // namespace tobas
