// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLineEdit>
#include <QPushButton>

#include <tobas_qt_tools/widgets/label.hpp>

#include "../base.hpp"

namespace tobas::gui::bm
{
class HostnameWidget : public BaseConfigWidget
{
  Q_OBJECT

  using self = HostnameWidget;

public:
  explicit HostnameWidget();

  const char* title() const override;

  void reset() override;

  bool onConnected() override;

private:
  QLineEdit* hostname_;
  qt::Label* warn_text_;

  QPushButton* write_button_;

  QString getHostname() const;

  bool writeHostnameFile(const QString& _hostname);
  bool writeHostsFile(const QString& _hostname);

private Q_SLOTS:
  void onHostnameChanged(const QString& _hostname);
  void onWriteButtonClicked();
};
}  // namespace tobas::gui::bm
