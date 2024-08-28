#include <iostream>

#include <tobas_std_tools/string.hpp>

using namespace std;

int main()
{
  const string valid_email = "example@example.com";
  const string invalid_email = "invalid@.com";

  cout << boolalpha;
  cout << valid_email << ": " << tobas_std::isValidEmail(valid_email) << endl;
  cout << invalid_email << ": " << tobas_std::isValidEmail(invalid_email) << endl;
}
