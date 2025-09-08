#include "tobas_path_tools/core.hpp"

#include <fstream>
#include <iostream>

using namespace std;
namespace fs = filesystem;

namespace path
{
bool isReadable(const fs::path& file_path)
{
  const ifstream ifs(file_path);
  return ifs.good();
}

bool isWritable(const fs::path& file_path)
{
  const ofstream ofs(file_path, ios::app);  // 既存ファイルを破壊しないようにappendモードで開く
  return ofs.good();
}

expected<void, string> createDirectories(const fs::path& dir_path, bool exist_ok)
{
  if (fs::is_directory(dir_path)) {
    if (exist_ok) {
      return {};
    }
    else {
      return unexpected("\"" + dir_path.string() + "\" already exists.");
    }
  }

  error_code ec;
  if (!fs::create_directories(dir_path, ec)) {
    return unexpected(ec.message());
  }

  return {};
}

expected<void, string> createFilePath(const fs::path& file_path, bool exist_ok)
{
  // ファイルの存在を確認
  if (fs::is_regular_file(file_path)) {
    if (exist_ok) {
      return {};
    }
    else {
      return unexpected("\"" + file_path.string() + "\" already exists.");
    }
  }

  // ファイルのパスからディレクトリ部分のみを取得
  const auto dir_path = fs::path(file_path).parent_path();

  // ファイルの親ディレクトリまでのパスを作成
  const auto create_dir_res = createDirectories(dir_path, true);
  if (!create_dir_res) {
    return unexpected(create_dir_res.error());
  }

  // 空のファイルを作成
  const ofstream file(file_path);
  if (!file) {
    return unexpected("Failed to create \"" + file_path.string() + "\".");
  }

  return {};
}

size_t computeDirectorySize(const fs::path& dir_path)
{
  if (!fs::is_directory(dir_path)) {
    cerr << dir_path << " does not exist." << endl;
    return 0;
  }

  size_t total_size = 0;

  // ディレクトリの内容を再帰的に走査
  for (const auto& entry : fs::recursive_directory_iterator(dir_path)) {
    if (fs::is_regular_file(entry.status())) {
      total_size += fs::file_size(entry.path());
    }
  }

  return total_size;
}

std::expected<void, std::string> clearDirectory(const fs::path& dir_path)
{
  if (!fs::is_directory(dir_path)) {
    return unexpected("\"" + dir_path.string() + "\" does not exist.");
  }

  for (const auto& entry : fs::directory_iterator(dir_path)) {
    fs::remove_all(entry);
  }

  return {};
}
}  // namespace path
