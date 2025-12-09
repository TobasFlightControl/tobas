#include "tobas_std_tools/debug.hpp"

#include <iostream>

#include "tobas_std_tools/ansi_text_styles.hpp"

using namespace std;

namespace tbs
{
void _printLocation(const char* file, int line)
{
  cout << GREEN_PREFIX << "Called from file " << file << ", line " << line << COLOR_RESET << endl;
}
}  // namespace tbs
