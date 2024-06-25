#include <iostream>

#include <tobas_path_tools/core.hpp>

using namespace std;

int main()
{
  static constexpr char path[] = "/tmp/this/is/the/file/created/by/create_file";

  if (!path::createFilePath(path))
  {
    cerr << "Failed to create " << path << "." << endl;
    return EXIT_FAILURE;
  }

  cout << path << " is created successfully." << endl;
  return EXIT_SUCCESS;
}
