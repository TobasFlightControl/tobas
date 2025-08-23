#include <iostream>

#include <tobas_crypt/yescrypt.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <password>" << endl;
    return EXIT_FAILURE;
  }

  const auto password = argv[1];

  tobas::crypt::Yescrypt crypt;
  const auto hash = crypt.crypt(password);
  cout << password << " -> " << hash << endl;
}
