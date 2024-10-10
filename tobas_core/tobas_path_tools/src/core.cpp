#include <fstream>
#include <iostream>

#include "../include/tobas_path_tools/core.hpp"

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

bool createDirectories(const fs::path& dir_path, bool exist_ok)
{
  if (fs::is_directory(dir_path))
  {
    if (exist_ok)
    {
      return true;
    }
    else
    {
      cerr << "\"" << dir_path << "\" already exists.";
      return false;
    }
  }

  return fs::create_directories(dir_path);
}

bool createFilePath(const fs::path& file_path, bool exist_ok)
{
  // ファイルの存在を確認
  if (fs::is_regular_file(file_path))
  {
    if (exist_ok)
    {
      return true;
    }
    else
    {
      cerr << "\"" << file_path << "\" already exists.";
      return false;
    }
  }

  // ファイルのパスからディレクトリ部分のみを取得
  const auto dir_path = fs::path(file_path).parent_path();

  // ディレクトリが存在しなければ作成
  if (!fs::is_directory(dir_path))
  {
    if (!fs::create_directories(dir_path))
    {
      cerr << "Failed to create \"" << dir_path << "\".";
      return false;
    }
  }

  // 空のファイルを作成
  const ofstream file(file_path);
  if (!file)
  {
    cerr << "Failed to create \"" << file_path << "\"." << endl;
    return false;
  }

  return true;
}

size_t computeDirectorySize(const fs::path& dir_path)
{
  size_t total_size = 0;

  // ディレクトリの内容を再帰的に走査
  for (const auto& entry : fs::recursive_directory_iterator(dir_path))
    if (fs::is_regular_file(entry.status()))
      total_size += fs::file_size(entry.path());

  return total_size;
}

void clearDirectory(const fs::path& dir_path)
{
  for (const auto& entry : fs::directory_iterator(dir_path))
    fs::remove_all(entry);
}
}  // namespace path
