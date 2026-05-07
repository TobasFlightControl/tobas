// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
/* throttled名前空間のトピック名を返す． */
std::string addThrotNS(const std::string& topic);

/* remote_interface名前空間のトピック名を返す． */
std::string addIfaceNS(const std::string& topic);
}  // namespace tobas
