#include <iostream>

#include <tobas_algorithm/crc.hpp>

using namespace std;

int main()
{
  const uint16_t poly = (1 << 12) | (1 << 5) | (1 << 0);
  algo::CRC16Left crc(poly, 0xFFFF, 0xFFFF);
  crc.initialize();

  const uint8_t data[] = "0123456789";
  cout << "CRC-16: " << hex << uppercase << crc.compute(data, sizeof(data) - 1) << endl;
}
