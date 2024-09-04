#pragma once

#include <filesystem>

namespace path
{
/* ファイルが読み取り可能な場合にtrueを返す． */
bool isReadable(const std::filesystem::path& file_path);

/* ファイルが書き込み可能な場合にtrueを返す． */
bool isWritable(const std::filesystem::path& file_path);

/* ファイル及び中間パスを作成する． */
bool createFilePath(const std::filesystem::path& file_path, bool exist_ok = true);
}  // namespace path
