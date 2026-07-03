// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
/* Return the topic name in the `throttled` namespace. */
std::string addThrotNS(const std::string& topic);

/* Return the topic name in the `remote_interface` namespace. */
std::string addIfaceNS(const std::string& topic);
}  // namespace tobas
