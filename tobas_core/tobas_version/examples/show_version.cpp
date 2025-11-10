#include <iostream>

#include "tobas_version/version.hpp"

using namespace std;

int main()
{
  cout << tobas::version::kMajor << "." << tobas::version::kMinor << "." << tobas::version::kPatch << endl;
}
