#include <iostream>

#include <tobas_string_tools/stream.hpp>
#include <tobas_wpa_supplicant/wpa_supplicant.hpp>

int main()
{
  constexpr char input_text[] = "country=JP\n"
                                "ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev\n"
                                "update_config=1\n"
                                "\n"
                                "# Network 1\n"
                                "network={\n"
                                "	ssid=\"SSID_1\"    \n"
                                "	 psk=\"PSK_1\"   \n"
                                "	  key_mgmt=WPA-PSK  \n"
                                "	   priority=1 \n"
                                "}\n"
                                "\n"
                                "\n"
                                "# Network 2\n"
                                "network={\n"
                                "	ssid=\"SSID_2\"\n"
                                "	psk=\"PSK_2\"\n"
                                "	key_mgmt=WPA-PSK\n"
                                "	priority=0\n"
                                "}";

  constexpr char output_path[] = "/tmp/wpa_supplicant.conf";

  tobas::wpa::Parser parser;
  tobas::wpa::Data data;
  if (!parser.parseFromText(input_text, data)) {
    return EXIT_FAILURE;
  }

  tobas::wpa::Exporter exporter;
  const auto text = exporter.exportText(data);
  if (!tobas::str::writeText(output_path, text)) {
    return EXIT_FAILURE;
  }

  std::cout << "Configuration file is saved as \"" << output_path << "\"." << std::endl;
  return EXIT_SUCCESS;
}
