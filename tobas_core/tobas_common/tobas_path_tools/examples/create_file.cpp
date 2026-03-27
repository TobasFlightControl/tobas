#include <iostream>

#include <tobas_path_tools/core.hpp>

using namespace std;

int main()
{
  static constexpr char path[] = "/tmp/this/is/the/file/created/by/create_file";

  const auto res = tobas::path::createFilePath(path);
  if (!res) {
    cerr << res.error() << endl;
    return EXIT_FAILURE;
  }

  cout << path << " is created successfully." << endl;
  return EXIT_SUCCESS;
}
