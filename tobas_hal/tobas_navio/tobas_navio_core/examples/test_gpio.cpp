#include <iostream>
#include <unistd.h>

#include <tobas_linux/file.hpp>
#include <tobas_navio_core/util.hpp>

using namespace std;

int main(int argc, char* argv[])
{
  if (navio::checkAPM())
  {
    cerr << "checkAPM() failed." << endl;
    return EXIT_FAILURE;
  }

  if (getuid())
  {
    cerr << "Not root." << endl;
    return EXIT_FAILURE;
  }

  if (argc != 2)
  {
    cerr << "Usage: " << argv[0] << " <Pin>" << endl;
    return EXIT_FAILURE;
  }

  uint32_t pin = *argv[1] - '0';  // char -> int
  uint32_t bcm = 500 + pin - 1;

  // Setup GPIO
  char export_path[] = "/sys/class/gpio/export";
  int err = linux::writeFile(export_path, "%u", bcm);
  if (err < 0 && err != -EBUSY)  // エクスポートの際の"Device or resource busy"は問題ない
  {
    cerr << "Failed to setup GPIO: " << bcm << endl;
    return EXIT_FAILURE;
  }

  // Write output
  string direction_path = "/sys/class/gpio/gpio" + to_string(bcm) + "/direction";
  if (linux::writeFile(direction_path.c_str(), "out") < 0)
  {
    cerr << "Failed to write output: " << bcm << endl;
    return EXIT_FAILURE;
  }

  // Output 0 and 1 alternately
  string value_path = "/sys/class/gpio/gpio" + to_string(bcm) + "/value";
  while (true)
  {
    // Set high
    if (linux::writeFile(value_path.c_str(), "1") < 0)
    {
      cerr << "Failed to set high: " << bcm << endl;
      return EXIT_FAILURE;
    }

    // sleep(1);

    // Set low
    if (linux::writeFile(value_path.c_str(), "0") < 0)
    {
      cerr << "Failed to set low: " << bcm << endl;
      return EXIT_FAILURE;
    }

    // sleep(1);
  }

  return EXIT_SUCCESS;
}
