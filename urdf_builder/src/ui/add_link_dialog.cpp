#include <utility>
#include <QtWidgets/QtWidgets>

#include "../../include/urdf_builder/ui/add_link_dialog.hpp"
#include "ui_add_link_dialog.h"

namespace urdf_builder
{
namespace ui
{
AddLinkDialog::AddLinkDialog(const QStringList& link_names, view_model::LinkViewModel& link_vm)
  : ui_(new Ui::AddLinkDialogUI()), link_vm_(link_vm)
{
  ui_->setupUi(this);

  ui_->JointParentLinkComboBox->addItems(link_names);
  if (!link_names.empty())
    link_vm_.joint()->parentLinkName(link_names.first());

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
}

void AddLinkDialog::JointNameLineEditTextChanged(const QString& text)
{
  link_vm_.joint()->name(text);
  link_vm_.sync();
}

void AddLinkDialog::JointParentComboBoxIndexChanged(int)
{
  link_vm_.joint()->parentLinkName(ui_->JointParentLinkComboBox->currentText());
  link_vm_.joint()->generateName();
  link_vm_.sync();

  ui_->JointNameLineEdit->setText(link_vm_.joint()->name());
}
}  // namespace ui
}  // namespace urdf_builder
