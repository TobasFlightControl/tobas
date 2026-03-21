#include <iostream>

#include <tobas_std_tools/geometry.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 4) {
    cerr << "Usage: " << argv[0] << " <Roll> <Pitch> <Yaw> [degree]" << endl;
    return EXIT_FAILURE;
  }

  const auto roll = tbs::deg2rad(atof(argv[1]));
  const auto pitch = tbs::deg2rad(atof(argv[2]));
  const auto yaw = tbs::deg2rad(atof(argv[3]));

  const auto [qx, qy, qz, qw] = tbs::quaternionFromEuler(roll, pitch, yaw);
  cout << "Hamilton: " << qw << ", " << qx << ", " << qy << ", " << qz << endl;

  return EXIT_SUCCESS;
}
