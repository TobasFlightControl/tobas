#pragma once

namespace tobas
{
namespace st
{
bool commandLineOptionExists(char** begin, char** end, const char* option);
char* getCommandLineOption(char** begin, char** end, const char* option);
}  // namespace st
}  // namespace tobas
