#include <bit>
#include <iostream>

#include <tobas_algorithm/binary.hpp>

using namespace std;

int main()
{
  constexpr auto src = 1.2345f;
  constexpr auto bits = bit_cast<uint32_t>(src);

  const auto res1 = bit_cast<float32_t>(bits);
  const auto res2 = algo::decodeR32(bits);

  if (res1 != res2) {
    cerr << "Failed to cast U32 to F32: " << res1 << " != " << res2 << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
