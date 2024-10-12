#pragma once

#include <QDialog>

#include <tobas_qt_tools/widgets/list_widget.hpp>

#include "./command.hpp"

namespace gui
{
namespace control_system
{
class AddCommandDialog : public QDialog
{
  Q_OBJECT

  using self = AddCommandDialog;
  using super = QDialog;

public:
  explicit AddCommandDialog(QWidget* parent);

  command_t selectedCommand() const;

private:
  qt::ListWidget* command_list_;
  command_t selected_command_;

private Q_SLOTS:
  void onOkClicked();
};
}  // namespace control_system
}  // namespace gui
