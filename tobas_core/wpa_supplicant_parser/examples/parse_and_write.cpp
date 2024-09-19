#include <iostream>

#include <wpa_supplicant_parser/parser.hpp>

using namespace std;

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

  wpa::WPASupplicantParser parser;

  if (!parser.parseFromText(input_text))
    return EXIT_FAILURE;

  if (!parser.write(output_path))
    return EXIT_FAILURE;

  cout << "Configuration file is saved as \"" << output_path << "\"." << endl;
  return EXIT_SUCCESS;
}
