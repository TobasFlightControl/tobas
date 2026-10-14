// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./data.hpp"

namespace tobas::wpa
{
class Exporter
{
public:
  explicit Exporter();

  std::string exportText(const Data& _src) const;

private:
  static const char* countryCodeToString(CountryCode _cc);
};
}  // namespace tobas::wpa
