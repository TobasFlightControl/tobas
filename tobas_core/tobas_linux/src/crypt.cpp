#include "tobas_linux/crypt.hpp"

#include <crypt.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>

#include <tobas_linux/error.hpp>

using namespace std;
using namespace chrono;

namespace linux
{
namespace
{
/* 乱数で crypt 互換 base64 のソルト文字列 (./0-9A-Za-z) を生成する． */
string makeSalt(size_t len)
{
  constexpr char kUrandomPath[] = "/dev/urandom";
  auto fd = ::open(kUrandomPath, O_RDONLY);
  if (fd < 0) {
    cerr << "Failed to open " << kUrandomPath << ": " << linux::strError() << endl;
    return {};
  }

  vector<unsigned char> buf(len);
  const auto n = ::read(fd, buf.data(), buf.size());
  ::close(fd);

  if (n != static_cast<ssize_t>(buf.size())) {
    cerr << "Failed to read urandom." << endl;
    return {};
  }

  constexpr char tbl[] = "./0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  string s;
  s.resize(len);
  for (size_t i = 0; i < len; ++i) {
    s[i] = tbl[buf[i] & 63];  // 0..63 -> 64種
  }

  return s;
}

/* /etc/shadow を行単位で読み込む． */
vector<string> readLines(const string& path)
{
  ifstream ifs(path);
  if (!ifs) {
    cerr << "Failed to open " << path << ": " << linux::strError() << endl;
    return {};
  }

  vector<string> lines;
  string line;
  while (getline(ifs, line)) {
    lines.push_back(line);
  }

  return lines;
}

/* コロン区切りを配列に変換する． */
vector<string> splitShadow(const string& line)
{
  vector<string> fields;
  size_t start = 0;

  while (true) {
    const auto pos = line.find(':', start);
    if (pos == string::npos) {
      fields.push_back(line.substr(start));
      break;
    }

    fields.push_back(line.substr(start, pos - start));
    start = pos + 1;
  }

  return fields;
}

/* 配列をコロン区切りに戻す． */
string joinShadow(const vector<string>& fields)
{
  ostringstream ss;
  for (size_t i = 0; i < fields.size(); ++i) {
    if (i) {
      ss << ':';
    }
    ss << fields[i];
  }
  return ss.str();
}

/* パスワードとソルトからハッシュを生成する． */
string crypt(const string& password, const string& salt)
{
  struct crypt_data data;
  memset(&data, 0, sizeof(data));

  const auto out = ::crypt_r(password.c_str(), salt.c_str(), &data);
  if (!out) {
    cerr << "crypt_r failed: " << linux::strError() << endl;
    return {};
  }

  return string(out);
}

/* ファイルを安全に上書きする． */
bool atomicOverwrite(const string& path, const string& content)
{
  // 既存のメタデータを保存
  struct stat st;
  if (stat(path.c_str(), &st) < 0) {
    cerr << "stat failed on " + path + ": " << linux::strError() << endl;
    return false;
  }

  // 同ディレクトリにテンポラリを作る
  const auto dir = path.substr(0, path.find_last_of('/'));
  auto tmp = dir + "/.shadow.tmp.XXXXXX";
  vector<char> tmpc(tmp.begin(), tmp.end());
  tmpc.push_back('\0');

  auto fd = ::mkstemp(tmpc.data());
  if (fd < 0) {
    cerr << "mkstemp failed: " << linux::strError() << endl;
    return false;
  }
  tmp.assign(tmpc.data());

  // パーミッション/オーナーを合わせる (安全のため 0640 で上書き)
  if (fchmod(fd, st.st_mode & 0640 ? st.st_mode : 0640) < 0) {
    cerr << "Failed to change mode." << endl;
    return false;
  }
  if (fchown(fd, st.st_uid, st.st_gid) < 0) {
    cerr << "Failed to change owner." << endl;
    return false;
  }

  // 書き込み
  const auto wr = ::write(fd, content.data(), content.size());
  if (wr != static_cast<ssize_t>(content.size())) {
    ::close(fd);
    ::unlink(tmp.c_str());
    cerr << "write failed" << endl;
    return false;
  }

  // 改行で終わっていなければ付与
  if (content.empty() || content.back() != '\n') {
    if (::write(fd, "\n", 1) != 1) {
      cerr << "write failed: " << linux::strError() << endl;
      return false;
    }
  }

  // ディスクへフラッシュ
  if (::fsync(fd) < 0) {
    ::close(fd);
    ::unlink(tmp.c_str());
    cerr << "fsync failed" << endl;
    return false;
  }
  ::close(fd);

  // 原子的に差し替え
  if (::rename(tmp.c_str(), path.c_str()) < 0) {
    ::unlink(tmp.c_str());
    cerr << "rename failed: " << linux::strError() << endl;
    return false;
  }

  return true;
}
}  // namespace

string crypt_sha512(const string& password, int rounds)
{
  // Create salt
  auto salt = makeSalt(16);
  if (salt.empty()) {
    return {};
  }
  salt = "$6$rounds=" + to_string(rounds) + "$" + salt;

  return crypt(password, salt);
}

string crypt_yescrypt(const string& password)
{
  // Create salt
  char salt[CRYPT_GENSALT_OUTPUT_SIZE]{};
  if (!crypt_gensalt_rn("$y$", 0, nullptr, 0, salt, sizeof(salt))) {
    cerr << "crypt_gensalt_rn failed: " << linux::strError() << endl;
    return {};
  }

  return crypt(password, salt);
}

bool setShadowPassword(const string& shadow_path, const string& username, const string& new_password)
{
  auto lines = readLines(shadow_path);
  if (lines.size() == 0) {
    return false;
  }

  // ハッシュを生成
  const auto hash = crypt_yescrypt(new_password);
  if (hash.empty()) {
    return false;
  }

  // 変更日を取得 (days since epoch)
  const auto days = duration_cast<hours>(system_clock::now().time_since_epoch()).count() / 24;

  // ユーザのログインパスワードのみ変更
  bool found = false;
  for (auto& line : lines) {
    // 空欄とコメント行をスキップ
    if (line.empty() || line[0] == '#') {
      continue;
    }

    auto fields = splitShadow(line);
    if (fields.empty()) {
      continue;
    }

    if (fields[0] == username) {
      // フィールド数が足りなければ埋める
      constexpr size_t kMinNumFields = 9;
      if (fields.size() < kMinNumFields) {
        fields.resize(kMinNumFields, "");
      }

      fields[1] = hash;             // ハッシュ
      fields[2] = to_string(days);  // 最終変更日
      line = joinShadow(fields);
      found = true;
      break;
    }
  }

  if (!found) {
    cerr << "user not found in shadow: " << username << endl;
    return false;
  }

  // 内容をまとめる
  ostringstream out;
  for (size_t i = 0; i < lines.size(); ++i) {
    out << lines[i];
    if (i + 1 < lines.size()) {
      out << endl;
    }
  }

  // ファイルを安全に上書き
  if (!atomicOverwrite(shadow_path, out.str())) {
    return false;
  }

  return true;
}
}  // namespace linux
