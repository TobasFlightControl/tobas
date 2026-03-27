#pragma once

#include <expected>

#include <QString>

namespace tobas
{
namespace qt
{
/* Get the absolute path to the resource directory of this package. */
QString getResourcePath();

/* e.g. hoge/fuga/piyo.ext -> piyo */
QString getBaseName(const QString& path);

/* e.g. ~/hoge/fuga -> /home/user/hoge/fuga */
QString expandUser(const QString& path);

/**
 * @brief テキストをタイムスタンプ付きのファイルに書き込む．
 *
 * @param content 書き込むテキスト
 * @param dir_path ファイルを保存するディレクトリ
 * @param prefix ファイル名の接頭語
 * @param suffix ファイル名の接尾語
 * @param ext ファイルの拡張子
 *
 * @return 作成されたファイルのフルパス
 */
std::expected<QString, QString> writeTimestampedFile(
  const QString& content,
  const QString& dir_path,
  const QString& prefix = "",
  const QString& suffix = "",
  const QString& ext = "txt");
}  // namespace qt
}  // namespace tobas
