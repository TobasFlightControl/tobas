#include <iostream>

#include <tobas_ssh_authkey/parse.hpp>
#include <tobas_ssh_authkey/prettify.hpp>
#include <tobas_string_tools/core.hpp>
#include <tobas_string_tools/stream.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <Path>" << endl;
    return EXIT_FAILURE;
  }

  const auto path = argv[1];

  string raw_text;
  if (!str::readText(path, raw_text)) {
    return EXIT_FAILURE;
  }

  const auto data = tobas::sak::parseLine(str::trim(raw_text));
  if (!data) {
    cerr << data.error() << endl;
    return EXIT_FAILURE;
  }

  const auto pretty_line = tobas::sak::prettify(data.value());
  if (!pretty_line) {
    cerr << pretty_line.error() << endl;
    return EXIT_FAILURE;
  }

  cout << pretty_line.value() << endl;
}
