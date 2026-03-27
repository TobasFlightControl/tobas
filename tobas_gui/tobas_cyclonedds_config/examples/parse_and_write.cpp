#include <iostream>

#include <tobas_cyclonedds_config/cyclonedds_config.hpp>
#include <tobas_string_tools/stream.hpp>

int main()
{
  constexpr char input_text[] = "<CycloneDDS>\n"
                                "  <Domain>\n"
                                "    <General>\n"
                                "      <Interfaces>\n"
                                "        <NetworkInterface name=\"wlan0\"/>\n"
                                "        <NetworkInterface name=\"eth0\"/>\n"
                                "      </Interfaces>\n"
                                "    </General>\n"
                                "  </Domain>\n"
                                "</CycloneDDS>";

  constexpr char output_path[] = "/tmp/cyclonedds.xml";

  tobas::cyclonedds::Data data;
  if (!tobas::cyclonedds::parseFromText(input_text, data)) {
    return EXIT_FAILURE;
  }

  const auto text = tobas::cyclonedds::exportText(data);
  if (!tobas::str::writeText(output_path, text)) {
    return EXIT_FAILURE;
  }

  std::cout << "Configuration file is saved as \"" << output_path << "\"." << std::endl;
  return EXIT_SUCCESS;
}
