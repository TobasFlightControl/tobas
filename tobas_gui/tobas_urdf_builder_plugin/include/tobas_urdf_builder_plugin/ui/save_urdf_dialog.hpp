#pragma once

#include <QFileDialog>
#include <QLineEdit>
#include <QPushButton>

namespace gui
{
namespace ub
{
namespace ui
{
class SaveUrdfDialog : public QFileDialog
{
  Q_OBJECT

public:
  explicit SaveUrdfDialog(QWidget* parent, const QString& dir);

protected:
  bool eventFilter(QObject* obj, QEvent* event) override;

private Q_SLOTS:
  void onLineEditTextChanged();

private:
  QPushButton* save_button_;
  QLineEdit* line_edit_;
};
}  // namespace ui
}  // namespace ub
}  // namespace gui
