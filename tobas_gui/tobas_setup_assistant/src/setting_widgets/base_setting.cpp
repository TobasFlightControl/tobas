// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/base_setting.hpp"

#include <QTimer>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/util.hpp>

namespace tobas::gui::sa
{
BaseSettingWidget::BaseSettingWidget()
{
  title_ = new QLabel();
  title_->setFont(qt::DefaultFont(cmn::kTitlePSize, QFont::Bold));
  title_->setAlignment(Qt::AlignTop);

  description_ = new qt::DescriptionWidget("", cmn::kBodyPSize);

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

void BaseSettingWidget::addWidget(QWidget* _widget, int _stretch)
{
  content_rows_->addWidget(_widget, _stretch);
}

void BaseSettingWidget::addWidgetCenter(QWidget* _widget, int _stretch)
{
  qt::addWidgetCenter(_widget, content_rows_, _stretch);
}

void BaseSettingWidget::addLayout(QLayout* _layout, int _stretch)
{
  content_rows_->addLayout(_layout, _stretch);
}

void BaseSettingWidget::addStretch(int _stretch)
{
  content_rows_->addStretch(_stretch);
}

void BaseSettingWidget::addSpacing(int _size)
{
  content_rows_->addSpacing(_size);
}

void BaseSettingWidget::initialize()
{
  title_->setText(title());
  description_->setText(description());
}
}  // namespace tobas::gui::sa
