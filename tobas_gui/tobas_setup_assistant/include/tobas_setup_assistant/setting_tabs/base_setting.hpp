// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include <QVBoxLayout>
#include <QWidget>

namespace tobas
{
namespace gui
{
namespace sa
{
class BaseSettingWidget : public QWidget
{
  Q_OBJECT

  using self = BaseSettingWidget;
  using super = QWidget;

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
  QVBoxLayout* content_rows_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
