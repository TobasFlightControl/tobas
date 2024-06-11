#include <filesystem>
#include <fstream>

#include "../include/tobas_std_tools/file.hpp"

using namespace std;

namespace tobas_std
{
bool fileExists(const string& file_path)
{
  const ifstream ifile(file_path);
  return ifile.good();
}

void createFilePath(const string& file_path)
{
  // ファイルのパスからディレクトリ部分のみを取得
  const auto dir_path = filesystem::path(file_path).parent_path();

  // ディレクトリが存在しなければ作成
  if (!filesystem::exists(dir_path))
    if (!filesystem::create_directories(dir_path))
      throw runtime_error("Failed to create directory " + dir_path.string());

  // 空のファイルを作成
  const ofstream file(file_path);
  if (!file)
    throw runtime_error("Failed to create file " + file_path);
}
}  // namespace tobas_std
