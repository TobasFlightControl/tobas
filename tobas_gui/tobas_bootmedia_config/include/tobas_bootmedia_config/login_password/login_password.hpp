// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

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
class LoginPasswordWidget : public BaseConfigWidget
{
  Q_OBJECT

  using self = LoginPasswordWidget;
  using super = BaseConfigWidget;

public:
  explicit LoginPasswordWidget();

  const char* title() const override;

  void reset() override;

  bool onConnected() override;

private:
  qt::PasswordEdit* pswd1_;
  qt::PasswordEdit* pswd2_;

  qt::Label* warn_text_;

  QPushButton* write_button_;

private Q_SLOTS:
  void onTextChanged();
  void onWriteButtonClicked();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
