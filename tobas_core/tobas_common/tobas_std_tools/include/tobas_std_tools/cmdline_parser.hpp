#pragma once

namespace tbs
{
bool commandLineOptionExists(char** begin, char** end, const char* option);
char* getCommandLineOption(char** begin, char** end, const char* option);
}  // namespace tbs
