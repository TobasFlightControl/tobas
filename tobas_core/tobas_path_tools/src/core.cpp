#include <filesystem>
#include <fstream>

#include "../include/tobas_path_tools/core.hpp"

using namespace std;

namespace path
{
bool isReadable(const string& file_path)
{
  const ifstream ifs(file_path);
  return ifs.good();
}

bool isWritable(const string& file_path)
{
  // 既存ファイルを破壊しないようにappendモードで開く
  const ofstream ofs(file_path, ios::app);
  return ofs.good();
}

bool createFilePath(const string& file_path)
{
  // ファイルのパスからディレクトリ部分のみを取得
  const auto dir_path = filesystem::path(file_path).parent_path();

  // ディレクトリが存在しなければ作成
  if (!filesystem::is_directory(dir_path))
    if (!filesystem::create_directories(dir_path))
      return false;

  // 空のファイルを作成
  const ofstream file(file_path);
  if (!file)
    return false;

  return true;
}
}  // namespace path
