// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_wpa_supplicant/wpa_supplicant.hpp>

#include "../base.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
class WifiClientWidget : public BaseConfigWidget
{
  Q_OBJECT

  using self = WifiClientWidget;
  using super = BaseConfigWidget;

public:
  explicit WifiClientWidget();

  const char* title() const override;

  void reset() override;

  bool onConnected() override;

protected:
  void changeEvent(QEvent* _event) override;

private:
  wpa::Data wpa_data_;
  wpa::Exporter wpa_exporter_;

  QPushButton* add_button_;
  QPushButton* remove_button_;
  QPushButton* clear_button_;

  qt::TableWidget* table_;

  QString getKeyMgmt(int _row) const;
  QString getSsid(int _row) const;
  QString getPsk(int _row) const;
  int getPriority(int _row) const;
  bool getHidden(int _row) const;

  void addRow(const QString& _key_mgmt, const QString& _ssid, const QString& _psk, int _priority, bool _hidden);

  /** Match the styles of embedded widgets to the other cells. */
  void updateTableCellStyles();

  /** Write all current table contents to the configuration file on the boot device. */
  bool writeCurrentConfig();

private Q_SLOTS:
  void onAddButtonClicked();
  void onRemoveButtonClicked();
  void onClearButtonClicked();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
