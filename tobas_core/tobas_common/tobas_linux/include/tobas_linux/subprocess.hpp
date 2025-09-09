#pragma once

#include <sys/types.h>

#include <string>
#include <vector>

namespace linux
{
pid_t createSubprocess(const std::vector<char*>& _argv);

/* サブプロセスでbashコマンドを実行する． */
pid_t createSubprocess(const std::string& command);
}  // namespace linux
