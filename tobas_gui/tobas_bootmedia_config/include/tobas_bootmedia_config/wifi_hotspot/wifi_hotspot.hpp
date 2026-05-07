// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLineEdit>
#include <QPushButton>

#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/widgets/password_edit.hpp>

#include "../base.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
class WifiHotspotWidget : public BaseConfigWidget
{
  Q_OBJECT

  using self = WifiHotspotWidget;
  using super = BaseConfigWidget;

public:
  explicit WifiHotspotWidget();

  const char* name() const override;
  const char* title() const override;

  void reset() override;

private:
  QLineEdit* ssid_;
  qt::PasswordEdit* psk_;

  qt::Label* warn_text_;

  QPushButton* write_button_;

  QString getSsid() const;
  QString getPsk() const;

  bool checkSsid(QString& msg) const;
  bool checkPsk(QString& msg) const;

  static bool isValidAsciiPsk(const QString& psk);
  static bool isValid64HexPsk(const QString& psk);

private Q_SLOTS:
  void onTextChanged();
  void onWriteButtonClicked();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
