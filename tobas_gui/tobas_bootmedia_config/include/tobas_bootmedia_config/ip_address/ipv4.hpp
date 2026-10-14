// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/ipv4_edit.hpp>

#include "./base.hpp"

namespace tobas::gui::bm
{
class IPv4Widget : public BaseNetworkWidget
{
  Q_OBJECT

  using self = IPv4Widget;

  /** Method index */
  enum Method : int
  {
    kAutomatic = 0,
    kManual = 1,
  };

  /** Prefix length index */
  enum PrefixLength : int
  {
    kSlash8 = 0,
    kSlash16 = 1,
    kSlash24 = 2,
    kSlash25 = 3,
    kSlash26 = 4,
    kSlash27 = 5,
    kSlash28 = 6,
    kSlash29 = 7,
    kSlash30 = 8,
    kSlash32 = 9,
  };

public:
  explicit IPv4Widget(const QString& _name);

  QString name() const override;

  void reset() override;

  bool load(const Network& _src) override;
  Network dump() const override;

private:
  const QString name_;

  qt::ComboBox* method_;
  qt::ComboBox* prefix_;
  qt::IPv4Edit* address_;
  qt::IPv4Edit* gateway_;

  void enableManualFields(bool _enabled);

  static std::expected<PrefixLength, QString> prefixValueToIndex(uint8_t _value);
  static uint8_t prefixIndexToValue(PrefixLength _index);

private Q_SLOTS:
  void onMethodChanged(int _index);
};
}  // namespace tobas::gui::bm
