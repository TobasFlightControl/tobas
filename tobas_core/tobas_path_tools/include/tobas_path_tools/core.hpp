#pragma once

#include <filesystem>

namespace path
{
/* ファイルが読み取り可能な場合にtrueを返す． */
bool isReadable(const std::filesystem::path& file_path);

/* ファイルが書き込み可能な場合にtrueを返す． */
bool isWritable(const std::filesystem::path& file_path);

/**
 * @brief 中間パスを含めてディレクトリを作成する．
 * fs::create_directoriesと異なり，exist_ok=trueならば既存の場合でもtrueを返す．
 */
bool createDirectories(const std::filesystem::path& dir_path, bool exist_ok = true);

/* ファイル及び中間パスを作成する． */
bool createFilePath(const std::filesystem::path& file_path, bool exist_ok = true);

/* ディレクトリに含まれる全てのファイルサイズ [Byte] の合計を計算する． */
size_t computeDirectorySize(const std::filesystem::path& dir_path);

/* ディレクトリ内のすべてのファイルやサブディレクトリを削除する． */
void clearDirectory(const std::filesystem::path& dir_path);
}  // namespace path
