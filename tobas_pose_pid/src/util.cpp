#include "./util.hpp"

#include <iostream>

using namespace std;

namespace tobas
{
bool checkIndex(int idx)
{
  if (idx < 0 || 3 <= idx) {
    cerr << "Index " << idx << " is out of range." << endl;
    return false;
  }

  return true;
}
}  // namespace tobas
