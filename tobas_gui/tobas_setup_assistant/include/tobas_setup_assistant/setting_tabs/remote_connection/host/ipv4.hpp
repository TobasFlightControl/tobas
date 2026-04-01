// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/ipv4_edit.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace rc
{
class IPv4Widget : public BaseHostWidget
{
  Q_OBJECT

  static constexpr char kAddressKey[] = "address";

public:
  explicit IPv4Widget();

  const char* label() const override;

  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  QString host() const override;

private:
  qt::IPv4Edit* ipv4_;
};
}  // namespace rc
}  // namespace sa
}  // namespace gui
}  // namespace tobas
