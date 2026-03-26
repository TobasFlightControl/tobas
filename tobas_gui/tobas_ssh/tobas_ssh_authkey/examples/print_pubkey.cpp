#include <iostream>

#include <tobas_ssh_authkey/export.hpp>
#include <tobas_ssh_authkey/parse.hpp>
#include <tobas_ssh_authkey/prettify.hpp>
#include <tobas_string_tools/core.hpp>
#include <tobas_string_tools/stream.hpp>

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <Path>" << std::endl;
    return EXIT_FAILURE;
  }

  const auto path = argv[1];

  std::string raw_text;
  if (!tobas::str::readText(path, raw_text)) {
    return EXIT_FAILURE;
  }

  const auto original_line = tobas::str::trim(raw_text);

  const auto data = tobas::ssh::ak::parseLine(original_line);
  if (!data) {
    std::cerr << data.error() << std::endl;
    return EXIT_FAILURE;
  }

  const auto exported_line = tobas::ssh::ak::exportLine(data.value());
  if (!exported_line) {
    std::cerr << exported_line.error() << std::endl;
    return EXIT_FAILURE;
  }

  const auto prettified_line = tobas::ssh::ak::prettify(data.value());
  if (!prettified_line) {
    std::cerr << prettified_line.error() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Original  : " << original_line << std::endl;
  std::cout << "Exported  : " << exported_line.value() << std::endl;
  std::cout << "Prettified: " << prettified_line.value() << std::endl;
}
