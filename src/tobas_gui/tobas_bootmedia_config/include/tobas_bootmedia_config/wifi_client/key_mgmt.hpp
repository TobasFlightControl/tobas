// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QMetaType>
#include <QString>

#include <tobas_wpa_supplicant/key_mgmt.hpp>

namespace tobas::gui::bm
{
/** Get the display label for a Wi-Fi authentication method. */
QString keyMgmtLabel(wpa::KeyMgmt _key_mgmt);
}  // namespace tobas::gui::bm

Q_DECLARE_METATYPE(tobas::wpa::KeyMgmt)
