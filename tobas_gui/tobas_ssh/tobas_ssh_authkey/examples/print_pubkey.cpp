#include <iostream>

#include <tobas_ssh_authkey/export.hpp>
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

  const auto original_line = str::trim(raw_text);

  const auto data = tobas::ssh::ak::parseLine(original_line);
  if (!data) {
    cerr << data.error() << endl;
    return EXIT_FAILURE;
  }

  const auto exported_line = tobas::ssh::ak::exportLine(data.value());
  if (!exported_line) {
    cerr << exported_line.error() << endl;
    return EXIT_FAILURE;
  }

  const auto prettified_line = tobas::ssh::ak::prettify(data.value());
  if (!prettified_line) {
    cerr << prettified_line.error() << endl;
    return EXIT_FAILURE;
  }

  cout << "Original  : " << original_line << endl;
  cout << "Exported  : " << exported_line.value() << endl;
  cout << "Prettified: " << prettified_line.value() << endl;
}
