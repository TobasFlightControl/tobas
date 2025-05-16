#include <iostream>

#include <tobas_string_tools/core.hpp>

using namespace std;

int main()
{
  const string valid_email = "example@example.com";
  const string invalid_email = "invalid@.com";

  cout << boolalpha;
  cout << valid_email << ": " << str::isValidEmail(valid_email) << endl;
  cout << invalid_email << ": " << str::isValidEmail(invalid_email) << endl;
  cout << noboolalpha;
}
