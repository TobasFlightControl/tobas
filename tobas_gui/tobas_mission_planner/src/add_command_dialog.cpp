#include <magic_enum.hpp>
#include <QPushButton>
#include <QVBoxLayout>

#include "tobas_mission_planner/add_command_dialog.hpp"

namespace gui
{
namespace mission_planner
{
AddCommandDialog::AddCommandDialog(QWidget* parent) : super(parent)
{
  setWindowTitle("Add Command");

  const auto rows = new QVBoxLayout();
  setLayout(rows);

  command_list_ = new qt::ListWidget();
  command_list_->setSelectionMode(QListWidget::SingleSelection);
  for (const auto& cmd : magic_enum::enum_values<command_t>())
    command_list_->addItem(commandToText(cmd));
  rows->addWidget(command_list_);

  const auto cols = new QHBoxLayout();
  rows->addLayout(cols);

  const auto cancel_button = new QPushButton("Cancel");
  connect(cancel_button, &QPushButton::clicked, this, &self::reject);
  cols->addWidget(cancel_button);

  const auto ok_button = new QPushButton("OK");
  connect(ok_button, &QPushButton::clicked, this, &self::onOkClicked);
  cols->addWidget(ok_button);
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
};  // namespace mission_planner
}  // namespace gui
