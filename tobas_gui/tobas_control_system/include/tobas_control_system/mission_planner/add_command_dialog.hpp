#pragma once

#include <QDialog>

#include <tobas_qt_tools/widgets/list_widget.hpp>

#include "./command.hpp"

namespace gui
{
namespace gcs
{
class AddCommandDialog : public QDialog
{
  Q_OBJECT

  using self = AddCommandDialog;
  using super = QDialog;

public:
  explicit AddCommandDialog(QWidget* parent);

  Command selectedCommand() const;

private:
  qt::ListWidget* command_list_;
  Command selected_command_;

private Q_SLOTS:
  void onOkClicked();
};
}  // namespace gcs
}  // namespace gui
