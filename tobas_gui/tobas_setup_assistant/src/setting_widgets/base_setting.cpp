#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_setup_assistant/setting_tabs/base_setting.hpp"

namespace gui
{
namespace setup_assistant
{
BaseSettingWidget::BaseSettingWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  header_rows_ = new QVBoxLayout();
  content_rows_ = new QVBoxLayout();
  rows->addLayout(header_rows_);
  rows->addLayout(content_rows_);
}

void BaseSettingWidget::initialize()
{
  addTitleAndDescription();
  onInit();
}

void BaseSettingWidget::addTitleAndDescription()
{
  const auto title_label = new QLabel(title());
  title_label->setFont(qt::DefaultFont(kTitlePSize, QFont::Bold));
  title_label->setAlignment(Qt::AlignTop);
  header_rows_->addWidget(title_label);

  const auto description_label = new qt::DescriptionWidget(description(), kBodyPSize);
  description_label->setFixedHeight(kDescriptionHeight);
  header_rows_->addWidget(description_label);
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
}  // namespace setup_assistant
}  // namespace gui
