#include <iostream>

#include <tobas_linux/core.hpp>

using namespace std;

int main()
{
  if (tobas::linux::isSuperUser()) {
    cout << "Running with super privileges." << endl;
  }
  else {
    cout << "Running with regular privileges." << endl;
  }
}
