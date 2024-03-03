#include <iostream>

#include "../include/tobas_std_tools/fstream.hpp"

using namespace std;

namespace tobas_std
{
bool fileExists(const string& filename)
{
  ifstream ifile(filename);
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
}  // namespace tobas_std
