#include "tobas_setup_assistant/setting_tabs/base_setting.hpp"

#include <QTimer>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/util.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
BaseSettingWidget::BaseSettingWidget()
{
  title_ = new QLabel();
  title_->setFont(tobas::qt::DefaultFont(cmn::kTitlePSize, QFont::Bold));
  title_->setAlignment(Qt::AlignTop);

  description_ = new tobas::qt::DescriptionWidget("", cmn::kBodyPSize);

  // Layout
  header_rows_ = new QVBoxLayout();
  header_rows_->addWidget(title_);
  header_rows_->addWidget(description_);

  content_rows_ = new QVBoxLayout();

  const auto rows = new QVBoxLayout();
  rows->addLayout(header_rows_);
  rows->addSpacing(30);
  rows->addLayout(content_rows_);

  setLayout(rows);

  QTimer::singleShot(0, this, &self::initialize);
}

void BaseSettingWidget::addWidget(QWidget* widget, int stretch)
{
  content_rows_->addWidget(widget, stretch);
}

void BaseSettingWidget::addWidgetCenter(QWidget* widget, int stretch)
{
  tobas::qt::addWidgetCenter(widget, content_rows_, stretch);
}

void BaseSettingWidget::addLayout(QLayout* layout, int stretch)
{
  content_rows_->addLayout(layout, stretch);
}

void BaseSettingWidget::addStretch(int stretch)
{
  content_rows_->addStretch(stretch);
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
}  // namespace tobas
