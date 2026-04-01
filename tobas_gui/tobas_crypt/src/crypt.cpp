// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_crypt/crypt.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>

#include <tobas_linux/error.hpp>

namespace ch = std::chrono;

namespace tobas
{
namespace crypt
{
namespace
{
/* /etc/shadow を行単位で読み込む． */
std::vector<std::string> readLines(const std::string& path)
{
  std::ifstream ifs(path);
  if (!ifs) {
    std::cerr << "Failed to open " << path << ": " << linux::strError() << std::endl;
    return {};
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(ifs, line)) {
    lines.push_back(line);
  }

  return lines;
}

/* コロン区切りを配列に変換する． */
std::vector<std::string> splitShadow(const std::string& line)
{
  std::vector<std::string> fields;
  size_t start = 0;

  while (true) {
    const auto pos = line.find(':', start);
    if (pos == std::string::npos) {
      fields.push_back(line.substr(start));
      break;
    }

    fields.push_back(line.substr(start, pos - start));
    start = pos + 1;
  }

  return fields;
}

/* 配列をコロン区切りに戻す． */
std::string joinShadow(const std::vector<std::string>& fields)
{
  std::ostringstream ss;
  for (size_t i = 0; i < fields.size(); ++i) {
    if (i) {
      ss << ':';
    }
    ss << fields[i];
  }
  return ss.str();
}

/* ファイルを安全に上書きする． */
bool atomicOverwrite(const std::string& path, const std::string& content)
{
  // 既存のメタデータを保存
  struct stat st;
  if (stat(path.c_str(), &st) < 0) {
    std::cerr << "stat failed on " + path + ": " << linux::strError() << std::endl;
    return false;
  }

  // 同ディレクトリにテンポラリを作る
  const auto dir = path.substr(0, path.find_last_of('/'));
  auto tmp = dir + "/.shadow.tmp.XXXXXX";
  std::vector<char> tmpc(tmp.begin(), tmp.end());
  tmpc.push_back('\0');

  auto fd = ::mkstemp(tmpc.data());
  if (fd < 0) {
    std::cerr << "mkstemp failed: " << linux::strError() << std::endl;
    return false;
  }
  tmp.assign(tmpc.data());

  // パーミッション/オーナーを合わせる (安全のため 0640 で上書き)
  if (fchmod(fd, st.st_mode & 0640 ? st.st_mode : 0640) < 0) {
    std::cerr << "Failed to change mode." << std::endl;
    return false;
  }
  if (fchown(fd, st.st_uid, st.st_gid) < 0) {
    std::cerr << "Failed to change owner." << std::endl;
    return false;
  }

  // 書き込み
  const auto wr = ::write(fd, content.data(), content.size());
  if (wr != static_cast<ssize_t>(content.size())) {
    ::close(fd);
    ::unlink(tmp.c_str());
    std::cerr << "write failed" << std::endl;
    return false;
  }

  // 改行で終わっていなければ付与
  if (content.empty() || content.back() != '\n') {
    if (::write(fd, "\n", 1) != 1) {
      std::cerr << "write failed: " << linux::strError() << std::endl;
      return false;
    }
  }

  // ディスクへフラッシュ
  if (::fsync(fd) < 0) {
    ::close(fd);
    ::unlink(tmp.c_str());
    std::cerr << "fsync failed" << std::endl;
    return false;
  }
  ::close(fd);

  // 原子的に差し替え
  if (::rename(tmp.c_str(), path.c_str()) < 0) {
    ::unlink(tmp.c_str());
    std::cerr << "rename failed: " << linux::strError() << std::endl;
    return false;
  }

  return true;
}
}  // namespace

bool setShadowPassword(
  const std::string& _shadow_path,
  const std::string& _username,
  const std::string& _new_password,
  const Crypt& _crypt)
{
  auto lines = readLines(_shadow_path);
  if (lines.empty()) {
    return false;
  }

  // ハッシュを生成
  const auto hash = _crypt.crypt(_new_password);
  if (hash.empty()) {
    return false;
  }

  // 変更日を取得 (days since epoch)
  const auto days = duration_cast<ch::hours>(ch::system_clock::now().time_since_epoch()).count() / 24;

  // ユーザのログインパスワードのみ変更
  bool found = false;
  for (auto& line : lines) {
    // 空欄とコメント行をスキップ
    if (line.empty() || line[0] == '#') {
      continue;
    }

    auto fields = splitShadow(line);
    if (fields.empty()) {
      continue;
    }

    if (fields[0] == _username) {
      // フィールド数が足りなければ埋める
      constexpr size_t kMinNumFields = 9;
      if (fields.size() < kMinNumFields) {
        fields.resize(kMinNumFields, "");
      }

      fields[1] = hash;                  // ハッシュ
      fields[2] = std::to_string(days);  // 最終変更日
      line = joinShadow(fields);
      found = true;
      break;
    }
  }

  if (!found) {
    std::cerr << "user not found in shadow: " << _username << std::endl;
    return false;
  }

  // 内容をまとめる
  std::ostringstream out;
  for (size_t i = 0; i < lines.size(); ++i) {
    out << lines[i];
    if (i + 1 < lines.size()) {
      out << std::endl;
    }
  }

  // ファイルを安全に上書き
  if (!atomicOverwrite(_shadow_path, out.str())) {
    return false;
  }

  return true;
}
}  // namespace crypt
}  // namespace tobas
