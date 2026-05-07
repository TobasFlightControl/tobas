// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./data.hpp"

namespace tobas
{
namespace wpa
{
class Parser
{
public:
  explicit Parser();

  bool parseFromText(const std::string& text, Data& dst);

private:
  static bool parseCountryCode(const std::string& src, CountryCode& dst);
};
}  // namespace wpa
}  // namespace tobas
