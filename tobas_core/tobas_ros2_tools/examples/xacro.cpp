#include <cmath>
#include <iostream>

#include <tobas_ros2_tools/xacro.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <Xacro Path>" << endl;
    return EXIT_FAILURE;
  }

  string urdf_text;
  if (!ros2::parseXacroFromPath(argv[1], urdf_text)) {
    return EXIT_FAILURE;
  }

  cout << "URDF Content:" << endl;
  cout << urdf_text << endl;

  return EXIT_SUCCESS;
}
