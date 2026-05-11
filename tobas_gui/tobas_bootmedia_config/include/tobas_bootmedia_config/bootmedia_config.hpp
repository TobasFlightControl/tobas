// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./base.hpp"
#include "./media_manager.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
class BootmediaConfigWidget : public QWidget
{
  Q_OBJECT

  using self = BootmediaConfigWidget;
  using super = QWidget;

  static constexpr int kTabWidth = 70;
  static constexpr int kTabHeight = 35;

public:
  explicit BootmediaConfigWidget();

  void reset();

protected:
  void closeEvent(QCloseEvent* event) override;

private:
  MediaManagerWidget* media_manager_;
  qt::VerticalTabWidget* tabs_;

  BaseConfigWidget* getWidget(int index);

  void setTabsEnabled(bool enabled);

private Q_SLOTS:
  void onMediaConnected();
  void onMediaDisconnected();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
