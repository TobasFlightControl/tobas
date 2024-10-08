#include <iostream>

#include <tobas_algorithm/crc.hpp>

using namespace std;

int main()
{
  const uint32_t poly = 0x04C11DB7;
  algo::CRC32Left crc(poly, 0xFFFFFFFF, 0x00000000);

  const uint8_t data[] = "Example data for CRC calculation";
  cout << "CRC-32: " << hex << uppercase << crc.compute(data, sizeof(data) - 1) << endl;
}
