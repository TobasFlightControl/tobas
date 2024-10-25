#include <vector>
#include <iostream>
#include <unistd.h>

#include "../include/tobas_gui_common/ros2_cli.hpp"

using namespace std;

namespace gui
{
namespace common
{
pid_t rosrun(const string& pkg, const string& exec, const string& name)
{
  // 呼び出し元のプロセスをクローン
  const auto pid = fork();
  if (pid < 0)
  {
    cerr << "Failed to clone the calling process." << endl;
    return -1;
  }

  // 子プロセスのPIDが帰ってきた場合は親プロセスの処理を続ける
  if (pid > 0)
    return pid;

  vector<char*> argv;
  argv.push_back(const_cast<char*>("ros2"));
  argv.push_back(const_cast<char*>("run"));
  argv.push_back(const_cast<char*>(pkg.c_str()));
  argv.push_back(const_cast<char*>(exec.c_str()));

  if (!name.empty())
  {
    const auto name_arg = "--ros-args --name " + name;
    argv.push_back(const_cast<char*>(name_arg.c_str()));
  }

  argv.push_back(nullptr);  // 引数リストの終端

  // 子プロセスをros2 runのプロセスに置き換える
  execvp("ros2", argv.data());
  return -1;
}

pid_t roslaunch(const string& pkg, const string& name, const map<string, string>& args)
{
  // 呼び出し元のプロセスをクローン
  const auto pid = fork();
  if (pid < 0)
  {
    cerr << "Failed to clone the calling process." << endl;
    return -1;
  }

  // 子プロセスのPIDが帰ってきた場合は親プロセスの処理を続ける
  if (pid > 0)
    return pid;

  vector<char*> argv;
  argv.push_back(const_cast<char*>("ros2"));
  argv.push_back(const_cast<char*>("launch"));
  argv.push_back(const_cast<char*>(pkg.c_str()));
  argv.push_back(const_cast<char*>(name.c_str()));

  for (const auto& arg : args)
  {
    const auto arg_str = arg.first + ":=" + arg.second;
    argv.push_back(const_cast<char*>(arg_str.c_str()));
  }

  argv.push_back(nullptr);  // 引数リストの終端

  // 別プロセスでroslaunchを実行
  if (execvp("ros2", argv.data()) < 0)
  {
    cerr << "Failed to execute ros2 launch" << endl;
    return -1;
  }

  // 子プロセスをros2 runのプロセスに置き換える
  execvp("ros2", argv.data());
  return -1;
}
}  // namespace common
}  // namespace gui
