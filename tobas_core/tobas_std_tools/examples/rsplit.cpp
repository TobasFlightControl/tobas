#include <iostream>

#include <tobas_std_tools/string.hpp>

using namespace std;

int main()
{
  const string input = "example/directory/file.txt";
  const auto output = tobas_std::rsplit(input, '/');

  cout << "Input : " << input << endl;
  cout << "Output: " << output.first << ", " << output.second << endl;
}
