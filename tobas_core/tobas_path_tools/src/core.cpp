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
  // 既存ファイルを破壊しないようにappendモードで開く
  const ofstream ofs(file_path, ios::app);
  return ofs.good();
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
  const auto dir_path = filesystem::path(file_path).parent_path();

  // ディレクトリが存在しなければ作成
  if (!filesystem::is_directory(dir_path))
  {
    if (!filesystem::create_directories(dir_path))
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
}  // namespace path
