#include <iostream>

#include <tobas_std_tools/stream.hpp>

using namespace std;

int main()
{
  const auto result = tobas_std::buildString("Number: ", 42, " and ", 3.14);
  cout << result << endl;
}
