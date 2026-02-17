#include "tobas_control_system/mission_planner/commands/base.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QStackedWidget>
#include <QTimer>

#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>

namespace gui
{
namespace ctrl
{
BaseCommandWidget::BaseCommandWidget()
{
  const auto root_rows = new QVBoxLayout();
  setLayout(root_rows);

  const auto header_cols = new QHBoxLayout();
  root_rows->addLayout(header_cols);

  label_ = new QLabel();
  label_->setFont(qt::DefaultFont(kLablePSize, QFont::Bold));
  header_cols->addWidget(label_);

  header_cols->addStretch();

  delete_button_ = new QPushButton("Delete");
  delete_button_->setStyleSheet("background-color: red");
  header_cols->addWidget(delete_button_);
  connect(delete_button_, &QPushButton::clicked, this, &self::onDeleteButtonClicked);

  const auto scroll_area = new qt::ScrollArea();
  root_rows->addWidget(scroll_area);

  const auto field_rows = new QVBoxLayout();
  scroll_area->setLayout(field_rows);

  scroll_area->setBackgroundTransparent();

  form_ = new qt::FormLayout();
  field_rows->addLayout(form_);

  field_rows->addStretch(1);  // フォームウィジェットを最小化

  // 純粋仮想関数を基底クラスのコンストラクタで呼ぶことはできないため，タイマーコールバックを使用．
  QTimer::singleShot(0, this, &self::initialize);
}

void BaseCommandWidget::addField(field::BaseFieldWidget* widget, bool overridable)
{
  const auto checkbox = new qt::CheckBox(widget->label());
  checkbox->setDisabledTextNormal();
  checkboxes_[widget] = checkbox;

  const auto stacked = new QStackedWidget();
  stacked->setStyleSheet("QStackedWidget { border: 0px; }");  // 外枠を消す
  stacked->addWidget(new QLabel("    Project Default"));
  stacked->addWidget(widget);

  if (overridable) {
    checkbox->setChecked(false);
    stacked->setCurrentIndex(0);
  }
  else {
    checkbox->setChecked(true);
    checkbox->setEnabled(false);
    stacked->setCurrentIndex(1);
  }

  form_->addVAlignedRow(checkbox, stacked);

  connect(checkbox, &QCheckBox::toggled, [stacked](bool checked) { stacked->setCurrentIndex((int)checked); });
  connect(widget, &field::BaseFieldWidget::updated, this, &self::onFieldUpdated);

  ++row_;
}

bool BaseCommandWidget::isChecked(field::BaseFieldWidget* widget) const
{
  return checkboxes_[widget]->isChecked();
}

void BaseCommandWidget::initialize()
{
  label_->setText(name());
}

void BaseCommandWidget::onFieldUpdated()
{
  Q_EMIT updated();
}

void BaseCommandWidget::onDeleteButtonClicked()
{
  Q_EMIT deleteButtonClicked();
}
}  // namespace ctrl
}  // namespace gui
