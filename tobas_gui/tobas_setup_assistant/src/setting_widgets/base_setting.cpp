#include <QTimer>

#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_setup_assistant/setting_tabs/base_setting.hpp"

namespace gui
{
namespace sa
{
BaseSettingWidget::BaseSettingWidget()
{
  title_ = new QLabel();
  title_->setFont(qt::DefaultFont(kTitlePSize, QFont::Bold));
  title_->setAlignment(Qt::AlignTop);

  description_ = new qt::DescriptionWidget("", kBodyPSize);
  description_->setFixedHeight(kDescriptionHeight);

  // Layout
  header_rows_ = new QVBoxLayout();
  header_rows_->addWidget(title_);
  header_rows_->addWidget(description_);

  content_rows_ = new QVBoxLayout();

  const auto rows = new QVBoxLayout();
  rows->addLayout(header_rows_);
  rows->addLayout(content_rows_);

  setLayout(rows);

  QTimer::singleShot(0, this, &self::initialize);
}

void BaseSettingWidget::addWidget(QWidget* widget)
{
  content_rows_->addWidget(widget);
}

void BaseSettingWidget::addWidgetCenter(QWidget* widget)
{
  qt::addWidgetCenter(widget, content_rows_);
}

void BaseSettingWidget::addLayout(QLayout* layout)
{
  content_rows_->addLayout(layout);
}

void BaseSettingWidget::addStretch()
{
  content_rows_->addStretch();
}

void BaseSettingWidget::addSpacing(int size)
{
  content_rows_->addSpacing(size);
}

void BaseSettingWidget::initialize()
{
  title_->setText(title());
  description_->setText(description());
}
}  // namespace sa
}  // namespace gui
