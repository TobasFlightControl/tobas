#pragma once

#include <string>
#include <vector>
#include <sys/types.h>

namespace linux
{
pid_t createSubprocess(const std::vector<char*>& _argv);

/* サブプロセスでbashコマンドを実行する． */
pid_t createSubprocess(const std::string& command);
}  // namespace linux
