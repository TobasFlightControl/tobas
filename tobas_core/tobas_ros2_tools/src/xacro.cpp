#include "../include/tobas_ros2_tools/xacro.hpp"

#include <iostream>
#include <fstream>

using namespace std;

namespace ros2
{
bool xacro(const string& xacro_path, string& urdf_content)
{
  // テンプレート文字列．末尾6文字がXでなければならない．
  char tmp_urdf_path[] = "/tmp/temp_urdf_XXXXXX";

  // mkstemp()はテンプレートのX部分をランダムな文字列に置き換え，一時ファイルを作成する．
  if (mkstemp(tmp_urdf_path) < 0) {
    cerr << "Failed to create temporary URDF path." << endl;
    return false;
  }

  // XACROを展開してURDFを作成
  const auto command = "xacro " + xacro_path + " > " + tmp_urdf_path;
  if (system(command.c_str()) != EXIT_SUCCESS) {
    cerr << "Failed to convert XACRO to URDF." << endl;
    return false;
  }
  cout << "Temporary URDF is created: " << tmp_urdf_path << endl;

  // 作成したURDFを読み取る
  ifstream file(tmp_urdf_path);
  if (!file) {
    cerr << "Failed to open file: " << tmp_urdf_path << endl;
    return false;
  }
  urdf_content.clear();
  urdf_content.append((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());

  return true;
}
}  // namespace ros2
