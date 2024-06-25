#pragma once

#include <QtWidgets/QtWidgets>

namespace urdf_builder
{
namespace ui
{
class StringInputDialog : public QDialog
{
  Q_OBJECT

public:
  explicit StringInputDialog(
    QWidget* parent,
    const QString& title,
    const QString& name,
    const QString& default_text = "",
    const QStringList& excludeds = QStringList());

  QString getText() const;

private Q_SLOTS:
  void LineEditTextChanged(const QString& text);

private:
  const QStringList& excludeds_;

  QLineEdit* line_edit_;
  QLabel* warn_label_;
  QDialogButtonBox* button_box_;

  void enableOkButton(bool enable);
};
}  // namespace ui
}  // namespace urdf_builder
