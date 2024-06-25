#include <iostream>

#include <tobas_path_tools/util.hpp>

using namespace std;

int main()
{
  const string str = "Hello, world!";

  if (path::starts_with(str, "Hello"))
    cout << "The string starts with 'Hello'" << endl;
  else
    cout << "The string does not start with 'Hello'" << endl;

  if (path::ends_with(str, "world!"))
    cout << "The string ends with 'world!'" << endl;
  else
    cout << "The string does not end with 'world!'" << endl;

  return EXIT_SUCCESS;
}
