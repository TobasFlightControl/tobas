#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <magic_enum/magic_enum.hpp>

#include "tobas_control_system/mission_planner/add_command_dialog.hpp"

namespace gui
{
namespace gcs
{
AddCommandDialog::AddCommandDialog(QWidget* parent) : super(parent)
{
  setWindowTitle("Add Command");

  command_list_ = new qt::ListWidget();
  command_list_->setSelectionMode(QListWidget::SingleSelection);
  for (const auto& cmd : magic_enum::enum_values<command_t>())
    command_list_->addItem(commandToText(cmd));

  const auto cancel_button = new QPushButton("Cancel");
  const auto ok_button = new QPushButton("OK");
  ok_button->setDefault(true);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(cancel_button);
  cols->addWidget(ok_button);

  const auto rows = new QVBoxLayout();
  rows->addWidget(command_list_);
  rows->addLayout(cols);

  setLayout(rows);

  // Connection
  connect(cancel_button, &QPushButton::clicked, this, &self::reject);
  connect(ok_button, &QPushButton::clicked, this, &self::onOkClicked);
}

command_t AddCommandDialog::selectedCommand() const
{
  return selected_command_;
}

void AddCommandDialog::onOkClicked()
{
  const auto selected_item = command_list_->selectedItem();
  if (selected_item == nullptr)
    return;

  selected_command_ = textToCommand(selected_item->text().toUtf8());
  accept();
}
};  // namespace gcs
}  // namespace gui
