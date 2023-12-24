#include <iostream>
#include <unistd.h>

#include <Common/Util.h>

using namespace std;

int main(int argc, char* argv[])
{
  if (check_apm())
  {
    cerr << "check_apm() failed." << endl;
    return 1;
  }

  if (getuid())
  {
    cerr << "Not root." << endl;
    return 1;
  }

  if (argc != 2)
  {
    cerr << "Usage: " << argv[0] << " <Pin>" << endl;
    return 1;
  }

  uint32_t pin = *argv[1] - '0';  // char -> int
  uint32_t bcm = 500 + pin - 1;

  // Setup GPIO
  char export_path[] = "/sys/class/gpio/export";
  int err = write_file(export_path, "%u", bcm);
  if (err < 0 && err != -EBUSY)  // エクスポートの際の"Device or resource busy"は問題ない
  {
    cerr << "Failed to setup GPIO: " << bcm << endl;
    return 1;
  }

  // Write output
  string direction_path = "/sys/class/gpio/gpio" + to_string(bcm) + "/direction";
  if (write_file(direction_path.c_str(), "out") < 0)
  {
    cerr << "Failed to write output: " << bcm << endl;
    return 1;
  }

  // Output 0 and 1 alternately
  string value_path = "/sys/class/gpio/gpio" + to_string(bcm) + "/value";
  while (true)
  {
    // Set high
    if (write_file(value_path.c_str(), "1") < 0)
    {
      cerr << "Failed to set high: " << bcm << endl;
      return 1;
    }

    // sleep(1);

    // Set low
    if (write_file(value_path.c_str(), "0") < 0)
    {
      cerr << "Failed to set low: " << bcm << endl;
      return 1;
    }

    // sleep(1);
  }
}
