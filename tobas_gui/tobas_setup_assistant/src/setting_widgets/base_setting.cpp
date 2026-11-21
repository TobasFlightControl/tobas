// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/base_setting.hpp"

#include <tobas_qt_tools/util.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
BaseSettingWidget::BaseSettingWidget()
{
  content_rows_ = new QVBoxLayout(this);
}

void BaseSettingWidget::addWidget(QWidget* widget, int stretch)
{
  content_rows_->addWidget(widget, stretch);
}

void BaseSettingWidget::addWidgetCenter(QWidget* widget, int stretch)
{
  qt::addWidgetCenter(widget, content_rows_, stretch);
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

}  // namespace sa
}  // namespace gui
}  // namespace tobas
