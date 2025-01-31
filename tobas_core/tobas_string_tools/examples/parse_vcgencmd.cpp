#include <iostream>

#include <tobas_string_tools/core.hpp>

using namespace std;

int main()
{
  const string vcgencmd_out = "frequency(48)=1500345728\n";
  const auto freq_str = str::deleteNl(str::split(vcgencmd_out, '=').back());
  const auto freq = stoul(freq_str);
  cout << "CPU frequency: " << freq << " [Hz]" << endl;
}
