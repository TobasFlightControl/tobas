#include <iostream>

#include <tobas_std_tools/file.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    cerr << "Usage: " << argv[0] << " <file_path>" << endl;
    return 1;
  }

  const auto path = argv[1];
  if (tobas_std::fileExists(path))
    cout << path << " exists." << endl;
  else
    cout << path << " does not exist." << endl;

  return 0;
}
