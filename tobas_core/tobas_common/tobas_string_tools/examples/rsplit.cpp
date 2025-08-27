#include <iostream>

#include <tobas_string_tools/core.hpp>

using namespace std;

int main()
{
  const string input = "example/directory/file.txt";
  const auto output = str::rsplit(input, '/');

  cout << "Input : " << input << endl;
  cout << "Output: " << output.first << ", " << output.second << endl;
}
