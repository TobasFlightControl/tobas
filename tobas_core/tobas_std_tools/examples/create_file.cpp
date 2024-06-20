#include <iostream>

#include <tobas_std_tools/file.hpp>

using namespace std;

int main()
{
  static constexpr char path[] = "/tmp/this/is/the/file/created/by/create_file";
  tobas_std::createFilePath(path);
  cout << path << " is created." << endl;
  return 0;
}
