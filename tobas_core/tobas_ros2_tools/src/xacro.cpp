#include "tobas_ros2_tools/xacro.hpp"

#include <fstream>
#include <iostream>

#include "tobas_ros2_tools/path.hpp"

namespace ros2
{
bool parseXacroFromPath(const std::string& xacro_path, std::string& urdf_text)
{
  // 一時的なURDFのパスを作成
  std::string tmp_urdf_path;
  if (createTemporalFile(tmp_urdf_path) < 0) {
    std::cerr << "Failed to create a temporal URDF path." << std::endl;
    return false;
  }

  // XACROを展開してURDFを作成
  const auto command = "xacro " + xacro_path + " > " + tmp_urdf_path;
  if (system(command.c_str()) != EXIT_SUCCESS) {
    std::cerr << "Failed to convert XACRO to URDF." << std::endl;
    return false;
  }
  std::cout << "Temporary URDF is created: " << tmp_urdf_path << std::endl;

  // 作成したURDFを読み取る
  std::ifstream file(tmp_urdf_path);
  if (!file) {
    std::cerr << "Failed to open file: " << tmp_urdf_path << std::endl;
    return false;
  }
  urdf_text.clear();
  urdf_text.append((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  return true;
}

bool parseXacroFromText(const std::string& xacro_text, std::string& urdf_text)
{
  // 一時的なXACROのパスを作成
  std::string tmp_xacro_path;
  const auto fd = createTemporalFile(tmp_xacro_path);
  if (fd < 0) {
    std::cerr << "Failed to create a temporal XACRO path." << std::endl;
    return false;
  }

  // 一時ファイルを開く
  const auto fp = ::fdopen(fd, "wb");  // fd は mkstemp の戻り値
  if (!fp) {
    std::cerr << "Failed to open " << tmp_xacro_path << "." << std::endl;
    return false;
  }

  // XACROの内容を一時ファイルに書き込む
  const auto xacro_size = xacro_text.size();
  const auto n = ::fwrite(xacro_text.data(), 1, xacro_size, fp);
  if (n != xacro_size) {
    std::cerr << "Tried to write " << xacro_size << " bytes, but only " << n << " bytes were written." << std::endl;
    return false;
  }

  // 一時ファイルを閉じる
  if (::fclose(fp) < 0) {
    std::cerr << "Failed to close " << tmp_xacro_path << "." << std::endl;
    return false;
  }

  // 保存したXACROからURDFを生成
  return parseXacroFromPath(tmp_xacro_path, urdf_text);
}  // namespace ros2
}  // namespace ros2
