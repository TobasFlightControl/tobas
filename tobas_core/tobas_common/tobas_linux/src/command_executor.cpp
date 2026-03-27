#include "tobas_linux/command_executor.hpp"

#include <iostream>
#include <memory>

using namespace std;

namespace tobas
{
namespace linux
{
CommandExecutor::CommandExecutor()
{
}

bool CommandExecutor::execute(string command)
{
  // 標準エラー出力を標準出力にリダイレクト
  // TODO: より複雑なリダイレクトコマンドに対応
  const auto pos = command.find('>');
  if (pos == string::npos) {
    command += " 2>&1";  // リダイレクトが無ければ末尾に追加
  }
  else {
    command.insert(pos, " 2>&1 1");  // 標準出力のみをファイル出力するよう途中に挿入
  }

  // コマンドを実行
  unique_ptr<FILE, int (*)(FILE*)> pipe(popen((command).c_str(), "r"), pclose);
  if (!pipe) {
    cerr << "popen() failed." << endl;
    return false;
  }

  // 出力を読み込む
  output_.clear();
  while (fgets(buffer_.data(), buffer_.size(), pipe.get())) {
    output_ += buffer_.data();
  }

  // 出力末尾のの改行コードを削除
  if (!output_.empty() && output_.back() == '\n') {
    output_.pop_back();
  }

  // 終了ステータスを取得
  const auto status = pclose(pipe.release());
  return status == EXIT_SUCCESS;
}
}  // namespace linux
}  // namespace tobas
