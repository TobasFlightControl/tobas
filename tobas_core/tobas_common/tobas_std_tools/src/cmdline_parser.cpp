#include "tobas_std_tools/cmdline_parser.hpp"

#include <string.h>

#include <cstddef>
#include <iostream>

using namespace std;

namespace tbs
{
bool commandLineOptionExists(char** begin, char** end, const char* option)
{
  for (size_t i = 0; i < (size_t)(end - begin); ++i) {
    if (strcmp(begin[i], option) == 0) {
      return true;
    }
  }
  return false;
}

char* getCommandLineOption(char** begin, char** end, const char* option)
{
  size_t idx = 0;
  size_t end_idx = (size_t)(end - begin);
  for (; idx < end_idx; ++idx) {
    if (strncmp(begin[idx], option, strlen(option)) == 0) {
      break;
    }
  }

  if (idx < end_idx - 1 && !begin[idx++]) {
    cerr << "Command line option \"" << option << "\" does not exist." << endl;
    return nullptr;
  }

  return begin[idx];
}
}  // namespace tbs
