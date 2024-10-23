#pragma once

namespace tobas_std
{
bool commandLineOptionExists(char** begin, char** end, const char* option);
char* getCommandLineOption(char** begin, char** end, const char* option);
}  // namespace tobas_std
