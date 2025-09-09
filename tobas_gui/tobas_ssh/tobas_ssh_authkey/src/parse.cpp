#include "tobas_ssh_authkey/parse.hpp"

#include <fstream>
#include <string>
#include <tobas_string_tools/core.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace ssh
{
namespace ak
{
std::expected<Data, std::string> parseLine(const std::string& line)
{
  Data res;

  // 空白で区切る
  const auto tokens = str::split(str::trim(line), ' ');
  if (tokens.size() < 2) {
    return std::unexpected("too few tokens");
  }

  // 鍵タイプを検出
  size_t key_type_idx = 0;
  for (size_t i = 0; i < tokens.size(); ++i) {
    res.key_type = ssh_key_type_from_name(tokens[i].c_str());
    if (res.key_type != SSH_KEYTYPE_UNKNOWN) {
      key_type_idx = i;
      break;
    }
  }
  if (res.key_type == SSH_KEYTYPE_UNKNOWN) {
    return std::unexpected("key type/base64 not found");
  }

  // 鍵データを取得
  const auto key_b64 = tokens[key_type_idx + 1];
  if (ssh_pki_import_pubkey_base64(key_b64.c_str(), res.key_type, &res.key) != SSH_OK) {
    return std::unexpected("libssh: import failed");
  }

  // 鍵データ以降をコメントとして結合
  for (size_t i = key_type_idx + 2; i < tokens.size(); ++i) {
    if (!res.comment.empty()) {
      res.comment.push_back(' ');
    }
    res.comment += tokens[i];
  }

  return res;
}

std::expected<std::vector<Data>, std::string> parseFile(const fs::path& path)
{
  std::ifstream file(path);
  if (!file) {
    return std::unexpected("Failed to open " + path.string() + ".");
  }

  size_t row = 0;
  std::string line;
  std::vector<Data> res;

  while (std::getline(file, line)) {
    ++row;

    // 空行やコメント行をスキップ
    if (line.empty() || line.starts_with('#')) {
      continue;
    }

    // 1行だけ解析
    const auto data = parseLine(line);
    if (!data) {
      file.close();
      return std::unexpected("Failed to parse line " + std::to_string(row) + ": " + data.error());
    }

    res.push_back(data.value());
  }

  file.close();
  return res;
}
}  // namespace ak
}  // namespace ssh
}  // namespace tobas
