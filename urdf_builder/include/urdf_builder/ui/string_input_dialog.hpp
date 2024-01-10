#pragma once

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QDialogButtonBox>

namespace urdf_builder
{
namespace ui
{
class StringInputDialog : public QDialog
{
  Q_OBJECT

public:
  explicit StringInputDialog(
    const QString& title,
    const QString& name,
    const QString& default_text = "",
    const QStringList& excludeds = QStringList(),
    const QString& warn_msg = "This is already used.",
    QWidget* parent = nullptr);

  QString getText() const;

private Q_SLOTS:
  void LineEditTextChanged(const QString& text);

private:
  const QStringList& excludeds_;
  QLineEdit* line_edit_;
  QLabel* warn_label_;
  QDialogButtonBox* button_box_;
};
}  // namespace ui
}  // namespace urdf_builder
