#include <gtest/gtest.h>

#include <tobas_drone_core/drone.hpp>

using namespace std;

TEST(TestSuite, TestCase)
{
  static constexpr char kFilePath[] = "/tmp/example.tbsdrn";

  tobas::Drone drone;

  ASSERT_TRUE(drone.save(kFilePath)) << "Failed to save drone.";
  cout << "Drone configurations are saved to \"" << kFilePath << "\"." << endl;

  ASSERT_TRUE(drone.load(kFilePath)) << "Failed to load drone.";
  cout << "Drone configurations are loaded successfully." << endl;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
