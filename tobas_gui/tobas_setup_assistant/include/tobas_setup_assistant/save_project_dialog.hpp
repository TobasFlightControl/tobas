#pragma once

#include <QFileDialog>
#include <QLineEdit>
#include <QPushButton>

namespace gui
{
namespace sa
{
class SaveProjectDialog : public QFileDialog
{
  Q_OBJECT

public:
  explicit SaveProjectDialog(QWidget* parent, const QString& dir, const QString& dflt_name);

protected:
  bool eventFilter(QObject* obj, QEvent* event) override;

private Q_SLOTS:
  void onLineEditTextChanged();

private:
  QPushButton* save_button_;
  QLineEdit* line_edit_;
};
}  // namespace sa
}  // namespace gui
