#include <memory>
#include <iostream>

#include "../include/tobas_linux/command_executor.hpp"

using namespace std;

namespace linux
{
CommandExecutor::CommandExecutor()
{
}

bool CommandExecutor::execute(const string& command)
{
  // 標準エラーを標準出力にリダイレクトしてコマンドを実行
  unique_ptr<FILE, int (*)(FILE*)> pipe(popen((command + " 2>&1").c_str(), "r"), pclose);
  if (pipe == nullptr)
  {
    cerr << "popen() failed." << endl;
    return false;
  }

  // 出力を読み込む
  output_.clear();
  while (fgets(buffer_.data(), buffer_.size(), pipe.get()))
    output_ += buffer_.data();

  // 出力末尾のの改行コードを削除
  output_.pop_back();

  // 終了ステータスを取得
  const auto status = pclose(pipe.release());
  return status == EXIT_SUCCESS;
}
}  // namespace linux
