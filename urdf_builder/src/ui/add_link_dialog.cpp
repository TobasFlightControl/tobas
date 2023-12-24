#include <utility>
#include <QMessageBox>

#include "../../include/urdf_builder/ui/add_link_dialog.hpp"
#include "ui_add_link_dialog.h"

namespace urdf_builder
{
namespace ui
{
AddLinkDialog::AddLinkDialog(view_model::LinkViewModelPtr vm, QWidget* parent)
  : QDialog(parent), ui_(new Ui::AddLinkDialogUI()), vm_(std::move(vm))
{
  ui_->setupUi(this);

  ui_->JointParentLinkComboBox->addItems(vm_->joint()->usedLinkNames());
  if (!vm_->joint()->usedLinkNames().empty())
    vm_->joint()->parentLinkName(vm_->joint()->usedLinkNames().first());

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

void AddLinkDialog::done(int code)
{
  if (code == QDialog::Rejected)
  {
    QDialog::done(code);
    return;
  }

  if (!vm_->isValid())
  {
    QMessageBox::warning(this, "ERROR", "No link name specified or name already exist");
    return;
  }

  QDialog::done(code);
}

void AddLinkDialog::LinkNameLineEditTextChanged(const QString& text)
{
  vm_->name(text);
  vm_->joint()->childLinkName(text);
  vm_->joint()->generateName();
  vm_->sync();

  ui_->JointNameLineEdit->setText(vm_->joint()->name());
}

void AddLinkDialog::JointNameLineEditTextChanged(const QString& text)
{
  vm_->joint()->name(text);
  vm_->sync();
}

void AddLinkDialog::JointParentComboBoxIndexChanged(int)
{
  vm_->joint()->parentLinkName(ui_->JointParentLinkComboBox->currentText());
  vm_->joint()->generateName();
  vm_->sync();

  ui_->JointNameLineEdit->setText(vm_->joint()->name());
}
}  // namespace ui
}  // namespace urdf_builder
