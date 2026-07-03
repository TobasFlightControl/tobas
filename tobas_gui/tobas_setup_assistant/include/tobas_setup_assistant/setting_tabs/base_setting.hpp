// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>
#include <QLabel>
#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>

#include "../param_getters/base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class BaseSettingWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = BaseSettingWidget;
  using super = qt::ScrollArea;

public:
  explicit BaseSettingWidget();

  virtual const char* name() const = 0;
  virtual const char* title() const = 0;
  virtual const char* description() const = 0;

  /* Update the internal state according to URDF changes. */
  virtual void updateInternalDataStructures() = 0;

  /* Return true when user settings have no problems. */
  virtual bool isValid() = 0;

  /* Write user settings. */
  virtual YAML::Node dump() const = 0;

  /* Load user settings. */
  virtual void load(const YAML::Node& node) = 0;

protected:
  void addWidget(QWidget* widget, int stretch = 0);
  void addWidgetCenter(QWidget* widget, int stretch = 0);
  void addLayout(QLayout* layout, int stretch = 0);
  void addStretch(int stretch = 0);
  void addSpacing(int size);

private:
  QLabel* title_;
  qt::DescriptionWidget* description_;

  QVBoxLayout* header_rows_;
  QVBoxLayout* content_rows_;

private Q_SLOTS:
  void initialize();
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
