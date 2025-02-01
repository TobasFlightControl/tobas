#include <iostream>

#include <tobas_algorithm/crc.hpp>

using namespace std;

int main()
{
  algo::CRC32Left crc(algo::CRC32Left::CRC_32, 0x00000000, 0x00000000);
  crc.initialize();

  const uint8_t data[] = "Example data for CRC calculation";
  cout << "CRC-32: " << hex << uppercase << crc.compute(data, sizeof(data) - 1) << endl;
}
