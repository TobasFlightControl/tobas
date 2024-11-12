#include <iostream>
#include <unistd.h>

#include "../include/tobas_linux/subprocess.hpp"

using namespace std;

namespace linux
{
pid_t createSubprocess(const vector<char*>& command)
{
  if (command.size() == 0)
  {
    cerr << "The size of command list is 0." << endl;
    return -1;
  }

  // 呼び出し元のプロセスをクローン
  // この時点で全く同じ内容のプロセスが2つできる
  const auto pid = fork();
  if (pid < 0)
  {
    cerr << "Failed to clone the calling process." << endl;
    return -1;
  }

  if (pid > 0)
  {
    // 子プロセスのPIDが帰ってきた場合は親プロセスの処理を続ける
    return pid;
  }
  else
  {
    // PIDが0だった場合は子プロセスなので，その内容を与えられたコマンドに置換する．
    cout << "Executing: ";
    for (const auto& cmd_elem : command)
      cout << cmd_elem << " ";
    cout << endl;

    auto argv = command;
    argv.push_back(nullptr);                 // 引数リストの終端
    return execvp(argv.at(0), argv.data());  // ここでブロッキング
  }
}
}  // namespace linux
