// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "../base.hpp"
#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
class IpAddressWidget : public BaseConfigWidget
{
  Q_OBJECT

  using self = IpAddressWidget;
  using super = BaseConfigWidget;

  static constexpr int kTabWidth = 135;
  static constexpr int kTabHeight = 45;

public:
  explicit IpAddressWidget();

  const char* title() const override;

  void reset() override;

  bool onConnected() override;

private:
  qt::TabWidget* tabs_;
  QPushButton* write_button_;

  BaseNetworkWidget* getWidget(int index);

  static std::string networkFilePath(const std::string& name);

private Q_SLOTS:
  void onWriteButtonClicked();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
