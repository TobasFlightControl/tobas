#include <fstream>
#include <filesystem>

namespace tobas_std
{
bool fileExists(const std::string& file_path);

std::string expandPath(const std::string& path);

bool createFilePath(const std::filesystem::path& file_path);
}  // namespace tobas_std
