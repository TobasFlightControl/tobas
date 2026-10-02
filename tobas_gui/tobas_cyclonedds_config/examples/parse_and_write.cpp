// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

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
                                "    <SharedMemory>\n"
                                "      <Enable>true</Enable>\n"
                                "      <LogLevel>warn</LogLevel>\n"
                                "    </SharedMemory>\n"
                                "  </Domain>\n"
                                "</CycloneDDS>";

  constexpr char output_path[] = "/tmp/cyclonedds.xml";

  const auto data = tobas::cyclonedds::parseFromText(input_text);
  if (!data) {
    std::cerr << data.error() << std::endl;
    return EXIT_FAILURE;
  }

  const auto text = tobas::cyclonedds::exportText(*data);
  if (!tobas::str::writeText(output_path, text)) {
    std::cerr << "Write text failed." << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Configuration file is saved as '" << output_path << "'." << std::endl;
  return EXIT_SUCCESS;
}
