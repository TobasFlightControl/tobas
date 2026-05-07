// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./data.hpp"

namespace tobas
{
namespace wpa
{
class Exporter
{
public:
  explicit Exporter();

  std::string exportText(const Data& src) const;

private:
  static const char* countryCodeToString(CountryCode cc);
};
}  // namespace wpa
}  // namespace tobas
