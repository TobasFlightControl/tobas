#include "../../include/urdf_builder/ui/urdf_builder_panel.hpp"
#include "../../include/urdf_builder/ui/add_link_dialog.hpp"
#include "ui_add_link_dialog.h"

namespace urdf_builder
{
namespace ui
{
AddLinkDialog::AddLinkDialog(
  URDFBuilderPanel* main,
  const QStringList& link_names,
  view_model::LinkViewModel& link_vm)
  : QDialog(main), main_(main), ui_(new Ui::AddLinkDialogUI()), link_vm_(link_vm)
{
  ui_->setupUi(this);

  ui_->JointParentLinkComboBox->addItems(link_names);
  if (!link_names.empty())
    link_vm_.joint()->parentLinkName(link_names.first());

  enableOkButton(false);

  connect(
    ui_->LinkNameLineEdit, SIGNAL(textChanged(const QString&)), this,
    SLOT(LinkNameLineEditTextChanged(const QString&)));
  connect(
    ui_->JointNameLineEdit, SIGNAL(textChanged(const QString&)), this,
    SLOT(JointNameLineEditTextChanged(const QString&)));
  connect(
    ui_->JointParentLinkComboBox, SIGNAL(currentIndexChanged(int)), this,
    SLOT(JointParentComboBoxIndexChanged(int)));
}

void AddLinkDialog::LinkNameLineEditTextChanged(const QString& text)
{
  link_vm_.name(text);
  link_vm_.joint()->childLinkName(text);
  link_vm_.joint()->generateName();
  link_vm_.sync();

  ui_->JointNameLineEdit->setText(link_vm_.joint()->name());

  checkValidity();
}

void AddLinkDialog::JointNameLineEditTextChanged(const QString& text)
{
  link_vm_.joint()->name(text);
  link_vm_.sync();

  checkValidity();
}

void AddLinkDialog::JointParentComboBoxIndexChanged(int)
{
  link_vm_.joint()->parentLinkName(ui_->JointParentLinkComboBox->currentText());
  link_vm_.joint()->generateName();
  link_vm_.sync();

  ui_->JointNameLineEdit->setText(link_vm_.joint()->name());
}

void AddLinkDialog::checkValidity()
{
  const auto link_name = ui_->LinkNameLineEdit->text();
  const auto joint_name = ui_->JointNameLineEdit->text();

  if (link_name.isEmpty())
  {
    ui_->WarnTextLabel->setText("Please set link name.");
    enableOkButton(false);
    return;
  }

  if (main_->linkNames().contains(link_name))
  {
    ui_->WarnTextLabel->setText("The specified link name is already used.");
    enableOkButton(false);
    return;
  }

  if (joint_name.isEmpty())
  {
    ui_->WarnTextLabel->setText("Please set joint name.");
    enableOkButton(false);
    return;
  }

  if (main_->jointNames().contains(joint_name))
  {
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
}  // namespace urdf_builder
