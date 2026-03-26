#include "tobas_urdf_builder_plugin/ui/add_link_dialog.hpp"

#include <QPushButton>

#include "ui_add_link_dialog.h"

#include "tobas_urdf_builder_plugin/ui/urdf_builder_panel.hpp"

namespace tobas
{
namespace gui
{
namespace ub
{
namespace ui
{
AddLinkDialog::AddLinkDialog(UrdfBuilderPanel* main, const QStringList& link_names, view_model::LinkViewModel& link_vm)
  : super(main), main_(main), ui_(new Ui::AddLinkDialogUI()), link_vm_(link_vm)
{
  ui_->setupUi(this);

  ui_->JointParentLinkComboBox->addItems(link_names);
  if (!link_names.empty()) {
    link_vm_.joint()->parentLinkName(link_names.first());
  }

  setWindowTitle("Add Link");  // setupUiの後に呼ぶ必要がある
  enableOkButton(false);

  connect(ui_->LinkNameLineEdit, &QLineEdit::textChanged, this, &self::onLinkNameLineEditTextChanged);
  connect(ui_->JointNameLineEdit, &QLineEdit::textChanged, this, &self::onJointNameLineEditTextChanged);
  connect(
    ui_->JointParentLinkComboBox,
    QOverload<int>::of(&QComboBox::currentIndexChanged),
    this,
    &self::onJointParentComboBoxIndexChanged);
}

void AddLinkDialog::onLinkNameLineEditTextChanged(const QString& text)
{
  link_vm_.name(text);
  link_vm_.sync();

  checkValidity();
}

void AddLinkDialog::onJointNameLineEditTextChanged(const QString& text)
{
  link_vm_.joint()->name(text);
  link_vm_.sync();

  checkValidity();
}

void AddLinkDialog::onJointParentComboBoxIndexChanged(int)
{
  link_vm_.joint()->parentLinkName(ui_->JointParentLinkComboBox->currentText());
  link_vm_.sync();
}

void AddLinkDialog::checkValidity()
{
  const auto link_name = ui_->LinkNameLineEdit->text();
  const auto joint_name = ui_->JointNameLineEdit->text();

  if (link_name.isEmpty()) {
    ui_->WarnTextLabel->setText("Please set link name.");
    enableOkButton(false);
    return;
  }

  if (main_->linkNames().contains(link_name)) {
    ui_->WarnTextLabel->setText("The specified link name is already used.");
    enableOkButton(false);
    return;
  }

  if (joint_name.isEmpty()) {
    ui_->WarnTextLabel->setText("Please set joint name.");
    enableOkButton(false);
    return;
  }

  if (main_->jointNames().contains(joint_name)) {
    ui_->WarnTextLabel->setText("The specified joint name is already used.");
    enableOkButton(false);
    return;
  }

  ui_->WarnTextLabel->clear();
  enableOkButton(true);
}

void AddLinkDialog::enableOkButton(bool enable)
{
  ui_->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(enable);
}
}  // namespace ui
}  // namespace ub
}  // namespace gui
}  // namespace tobas
