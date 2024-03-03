#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>

#include "../../include/urdf_builder/ui/string_input_dialog.hpp"

namespace urdf_builder
{
namespace ui
{
StringInputDialog::StringInputDialog(
  const QString& title,
  const QString& name,
  const QString& default_text,
  const QStringList& excludes,
  const QString& warn_msg,
  QWidget* parent)
  : QDialog(parent), excludeds_(excludes)
{
  setWindowTitle(title);

  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto cols = new QHBoxLayout();
  rows->addLayout(cols);

  const auto label = new QLabel();
  label->setText(name);
  cols->addWidget(label);

  line_edit_ = new QLineEdit();
  line_edit_->setText(default_text);
  connect(
    line_edit_, SIGNAL(textChanged(const QString&)), this,
    SLOT(LineEditTextChanged(const QString&)));
  cols->addWidget(line_edit_);

  warn_label_ = new QLabel();
  warn_label_->setText(warn_msg);
  warn_label_->setStyleSheet("QLabel { color: red; }");
  warn_label_->setVisible(false);
  rows->addWidget(warn_label_);

  button_box_ = new QDialogButtonBox();
  button_box_->setOrientation(Qt::Horizontal);
  button_box_->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);
  QObject::connect(button_box_, SIGNAL(accepted()), this, SLOT(accept()));
  QObject::connect(button_box_, SIGNAL(rejected()), this, SLOT(reject()));
  rows->addWidget(button_box_);
}

QString StringInputDialog::getText() const
{
  return line_edit_->text();
}

void StringInputDialog::LineEditTextChanged(const QString& text)
{
  if (excludeds_.contains(text))
  {
    warn_label_->setVisible(true);
    button_box_->button(QDialogButtonBox::Ok)->setEnabled(false);
  }
  else
  {
    warn_label_->setVisible(false);
    button_box_->button(QDialogButtonBox::Ok)->setEnabled(true);
  }
}
}  // namespace ui
}  // namespace urdf_builder
