#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/font.hpp>

#include "tobas_setup_assistant/setting_widgets/base_setting.hpp"

namespace gui
{
namespace setup_assistant
{
BaseSettingWidget::BaseSettingWidget(SetupAssistant* main) : main_(main)
{
  rows_ = new QVBoxLayout();
  setLayout(rows_);
}

void BaseSettingWidget::initialize()
{
  addTitleAndDescription();
  onInit();
  rows_->addStretch();
}

void BaseSettingWidget::addTitleAndDescription()
{
  auto title_label = new QLabel(title());
  title_label->setFont(qt::DefaultFont(kTitlePSize, QFont::Bold));
  title_label->setAlignment(Qt::AlignTop);
  rows_->addWidget(title_label);

  auto description_label = new qt::DescriptionWidget(description(), kBodyPSize);
  description_label->setFixedHeight(kDescriptionHeight);
  rows_->addWidget(description_label);
}
}  // namespace setup_assistant
}  // namespace gui
