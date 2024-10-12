#include <iostream>

#include <tobas_std_tools/string.hpp>

using namespace std;

int main()
{
  const string input = "This is a test. Testing is fun.";
  const auto output = tobas_std::replace(input, "is", "was");

  cout << "Input : " << input << endl;
  cout << "Output: " << output << endl;
}
