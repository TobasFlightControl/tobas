#include "../../include/tobas_urdf_builder_plugin/ui/string_input_dialog.hpp"

namespace gui
{
namespace urdf_builder
{
namespace ui
{
StringInputDialog::StringInputDialog(
  QWidget* parent,
  const QString& title,
  const QString& name,
  const QString& default_text,
  const QStringList& excludes)
  : QDialog(parent), excludeds_(excludes)
{
  setWindowTitle(title);

  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto cols = new QHBoxLayout();
  rows->addLayout(cols);

  const auto label = new QLabel(name);
  cols->addWidget(label);

  line_edit_ = new QLineEdit(default_text);
  connect(line_edit_, &QLineEdit::textChanged, this, &self::LineEditTextChanged);
  cols->addWidget(line_edit_);

  warn_label_ = new QLabel();
  warn_label_->setStyleSheet("QLabel { color: red; }");
  rows->addWidget(warn_label_);

  button_box_ = new QDialogButtonBox();
  button_box_->setOrientation(Qt::Horizontal);
  button_box_->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);
  enableOkButton(false);
  connect(button_box_, &QDialogButtonBox::accepted, this, &self::accept);
  connect(button_box_, &QDialogButtonBox::rejected, this, &self::reject);
  rows->addWidget(button_box_);
}

QString StringInputDialog::getText() const
{
  return line_edit_->text();
}

void StringInputDialog::LineEditTextChanged(const QString& text)
{
  if (text.isEmpty())
  {
    warn_label_->setText("Please set text.");
    enableOkButton(false);
    return;
  }

  if (excludeds_.contains(text))
  {
    warn_label_->setText("This is already used.");
    enableOkButton(false);
    return;
  }

  warn_label_->clear();
  enableOkButton(true);
}

void StringInputDialog::enableOkButton(bool enable)
{
  button_box_->button(QDialogButtonBox::Ok)->setEnabled(enable);
}
}  // namespace ui
}  // namespace urdf_builder
}  // namespace gui
