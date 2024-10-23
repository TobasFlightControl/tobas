#include <iostream>

#include <tobas_std_tools/string.hpp>

using namespace std;

int main()
{
  const string input = "This is a test: x^2 + y^3 = z^4.";
  const string output = tobas_std::convertToSuperscript(input);

  cout << "Input : " << input << endl;
  cout << "Output: " << output << endl;
}
