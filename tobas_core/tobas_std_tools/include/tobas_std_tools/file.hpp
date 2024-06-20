#include <string>

namespace tobas_std
{
bool isReadable(const std::string& file_path);
bool isWritable(const std::string& file_path);
void createFile(const std::string& file_path);
}  // namespace tobas_std
