#include <iostream>
#include <unistd.h>

#include "../include/tobas_linux/subprocess.hpp"

using namespace std;

namespace linux
{
pid_t createSubprocess(const vector<char*>& _argv)
{
  if (_argv.size() == 0)
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
    for (const auto& cmd_elem : _argv)
      cout << cmd_elem << " ";
    cout << endl;

    auto argv = _argv;
    argv.push_back(nullptr);                 // 引数リストの終端
    return execvp(argv.at(0), argv.data());  // ここでブロッキング
  }
}

pid_t createSubprocess(const string& command)
{
  // XXX: const_castはコピーをとらないため，プロセス作成時にコマンドのメモリが有効なことを保証する必要がある．
  vector<char*> argv;
  argv.push_back(const_cast<char*>("/bin/bash"));
  argv.push_back(const_cast<char*>("-c"));
  argv.push_back(const_cast<char*>(command.c_str()));

  return createSubprocess(argv);
}
}  // namespace linux
