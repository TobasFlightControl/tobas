#pragma once

#include <string>
#include <vector>

namespace gazebo
{
class CsvReader
{
public:
  static bool readCsvFile(std::string file_name, std::vector<std::vector<double>>& datas);
};
}  // namespace gazebo
