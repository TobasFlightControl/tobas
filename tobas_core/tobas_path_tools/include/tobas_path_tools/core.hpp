#pragma once

#include <string>

namespace path
{
/* ファイルが読み取り可能な場合にtrueを返す． */
bool isReadable(const std::string& file_path);

/* ファイルが書き込み可能な場合にtrueを返す． */
bool isWritable(const std::string& file_path);

/* ファイル及び中間パスを作成する． */
bool createFilePath(const std::string& file_path);
}  // namespace path
