#include <filesystem>
#include <fstream>

#include "../include/tobas_std_tools/file.hpp"

using namespace std;

namespace tobas_std
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

void createFile(const string& file_path)
{
  // ファイルのパスからディレクトリ部分のみを取得
  const auto dir_path = filesystem::path(file_path).parent_path();

  // ディレクトリが存在しなければ作成
  if (!filesystem::is_directory(dir_path))
    if (!filesystem::create_directories(dir_path))
      throw runtime_error("Failed to create directory " + dir_path.string());

  // 空のファイルを作成
  const ofstream file(file_path);
  if (!file)
    throw runtime_error("Failed to create file " + file_path);
}
}  // namespace tobas_std
