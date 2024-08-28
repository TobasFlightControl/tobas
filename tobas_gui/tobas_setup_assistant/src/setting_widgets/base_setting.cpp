#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/font.hpp>

#include "tobas_setup_assistant/setting_widgets/base_setting.hpp"

namespace gui
{
namespace setup_assistant
{
BaseSettingWidget::BaseSettingWidget(SetupAssistant* main) : main_(main)
{
  auto rows = new QVBoxLayout();
  header_rows_ = new QVBoxLayout();
  content_rows_ = new QVBoxLayout();

  setLayout(rows);
  rows->addLayout(header_rows_);
  rows->addLayout(content_rows_);
  rows->addStretch();
}

void BaseSettingWidget::initialize()
{
  addTitleAndDescription();
  onInit();
}

void BaseSettingWidget::addTitleAndDescription()
{
  auto title_label = new QLabel(title());
  title_label->setFont(qt::DefaultFont(kTitlePSize, QFont::Bold));
  title_label->setAlignment(Qt::AlignTop);
  header_rows_->addWidget(title_label);

  auto description_label = new qt::DescriptionWidget(description(), kBodyPSize);
  description_label->setFixedHeight(kDescriptionHeight);
  header_rows_->addWidget(description_label);
}

void BaseSettingWidget::addWidget(QWidget* widget)
{
  content_rows_->addWidget(widget);
}

void BaseSettingWidget::addLayout(QLayout* layout)
{
  content_rows_->addLayout(layout);
}
}  // namespace setup_assistant
}  // namespace gui
