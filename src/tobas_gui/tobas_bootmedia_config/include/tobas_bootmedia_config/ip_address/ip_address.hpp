// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>
#include <QString>

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "../base.hpp"
#include "./base.hpp"

namespace tobas::gui::bm
{
class IpAddressWidget : public BaseConfigWidget
{
  Q_OBJECT

  using self = IpAddressWidget;

public:
  explicit IpAddressWidget();

  const char* title() const override;

  void reset() override;

  bool onConnected() override;

private:
  qt::TabWidget* tabs_;
  QPushButton* write_button_;

  BaseNetworkWidget* getWidget(int _index);

private Q_SLOTS:
  void onWriteButtonClicked();
};
}  // namespace tobas::gui::bm
