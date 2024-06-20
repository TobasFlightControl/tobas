#include <iostream>

#include <tobas_std_tools/unix.hpp>

using namespace std;

int main()
{
  if (tobas_std::isSuperUser())
    cout << "Running with super privileges." << endl;
  else
    cout << "Running with regular privileges." << endl;
}
