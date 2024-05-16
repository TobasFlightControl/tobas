#include <filesystem>
#include <fstream>
#include <iostream>

#include "../include/tobas_std_tools/file.hpp"

using namespace std;

namespace tobas_std
{
bool fileExists(const string& file_path)
{
  const ifstream ifile(file_path);
  return ifile.good();
}

string expandPath(const string& path)
{
  if (path.size() > 0 && path[0] == '~')
  {
    const auto home_dir = getenv("HOME");
    if (home_dir != nullptr)
      return string(home_dir) + path.substr(1);
    else
      cerr << "HOME environment variable not set." << endl;
  }

  return path;
}

bool createFilePath(const string& file_path)
{
  // ファイルのパスからディレクトリ部分のみを取得
  const auto dir_path = filesystem::path(file_path).parent_path();

  // ディレクトリが存在しなければ作成
  if (!filesystem::exists(dir_path) && !filesystem::create_directories(dir_path))
    return false;

  // 空のファイルを作成
  const ofstream file(file_path);

  if (!file)
    return false;

  return true;
}
}  // namespace tobas_std
