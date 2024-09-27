#include <QTimer>

#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_control_system/commands/base.hpp"

namespace gui
{
namespace control_system
{
BaseCommandWidget::BaseCommandWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  label_ = new QLabel();
  label_->setFont(qt::DefaultFont(kLablePSize, QFont::Bold));
  qt::addWidgetCenter(label_, rows);

  rows->addSpacing(30);

  form_ = new qt::FormLayout();
  rows->addLayout(form_);

  rows->addStretch();

  delete_button_ = new QPushButton("Delete Command");
  delete_button_->setStyleSheet("background-color: red");
  connect(delete_button_, &QPushButton::clicked, this, &self::onDeleteButtonClicked);
  qt::addWidgetCenter(delete_button_, rows);

  // 純粋仮想関数を基底クラスのコンストラクタで呼ぶことはできないため，タイマーコールバックを使用．
  QTimer::singleShot(0, this, &self::initialize);
}

void BaseCommandWidget::addField(field::BaseField* field)
{
  connect(field, &field::BaseField::updated, this, &self::onFieldUpdated);
  form_->addRow(field->label(), field);
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
}  // namespace control_system
}  // namespace gui
