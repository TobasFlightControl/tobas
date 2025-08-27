#include <iostream>

#include <tobas_string_tools/core.hpp>

using namespace std;

int main()
{
  const string input = "This is a test: x^2 + y^3 = z^4.";
  const string output = str::convertToSuperscript(input);

  cout << "Input : " << input << endl;
  cout << "Output: " << output << endl;
}
