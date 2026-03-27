#pragma once

#include <QDialog>
#include <QDialogButtonBox>
#include <QLabel>
#include <QLineEdit>

namespace tobas
{
namespace gui
{
namespace ub
{
namespace ui
{
class StringInputDialog : public QDialog
{
  Q_OBJECT

  using self = StringInputDialog;
  using super = QDialog;

public:
  explicit StringInputDialog(
    QWidget* parent,
    const QString& title,
    const QString& name,
    const QString& default_text = "",
    const QStringList& excludeds = QStringList());

  QString getText() const;

private Q_SLOTS:
  void onLineEditTextChanged(const QString& text);

private:
  const QStringList& excludeds_;

  QLineEdit* line_edit_;
  QLabel* warn_label_;
  QDialogButtonBox* button_box_;

  void enableOkButton(bool enable);
};
}  // namespace ui
}  // namespace ub
}  // namespace gui
}  // namespace tobas
