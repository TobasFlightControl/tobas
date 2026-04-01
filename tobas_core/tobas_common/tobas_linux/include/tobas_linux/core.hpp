// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

namespace tobas
{
namespace linux
{
/* ユーザ名を取得する． */
std::string userName();

/* ホームディレクトリを取得する． */
std::filesystem::path homeDir();

/* ホームディレクトリを絶対パスに変換する． */
std::filesystem::path expandUser(const std::string& path);

/* プログラムがRoot権限で実行されている場合にTrueを返す． */
bool isSuperUser() noexcept;
}  // namespace linux
}  // namespace tobas
