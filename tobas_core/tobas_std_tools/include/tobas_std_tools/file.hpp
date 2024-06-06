#include <string>

namespace tobas_std
{
bool fileExists(const std::string& file_path);

/* ホームディレクトリを絶対パスに変換する． */
std::string expandPath(const std::string& path);

bool createFilePath(const std::string& file_path);
}  // namespace tobas_std
