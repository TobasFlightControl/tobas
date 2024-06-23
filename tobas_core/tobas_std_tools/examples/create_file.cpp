#include <iostream>

#include <tobas_std_tools/file.hpp>

using namespace std;

int main()
{
  static constexpr char path[] = "/tmp/this/is/the/file/created/by/create_file";

  if (!tobas_std::createFilePath(path))
  {
    cerr << "Failed to create " << path << "." << endl;
    return EXIT_FAILURE;
  }

  cout << path << " is created successfully." << endl;
  return EXIT_SUCCESS;
}
