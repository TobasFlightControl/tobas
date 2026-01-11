#pragma once

#include <netinet/in.h>

#include <string>

namespace ntrip
{
class NtripClient
{
public:
  explicit NtripClient();
  bool initialize(
    const char* server_ip,
    const int& server_port,
    const char* mount_point,
    const char* user_name,
    const char* password,
    const double& latitude,
    const double& longitude);

private:
  static constexpr int kTimeout = 5;  // 5 second
  static constexpr size_t kChunkSize = 1024;
  static constexpr size_t kMountPointsToShow = 5;
  enum MountPointStatus : uint8_t
  {
    SUCCESS,                // ICY 200 OKが含まれるデータを受信，接続成功
    GET_SOURCE_TABLE,       // SOURCETABLE 200 OKが含まれるsource tableのデータを受信，RTK2goへの接続はできているが，
                            // mount pointへの接続はできていない
    AUTHORIZATION_FAILURE,  // 401が含まれるデータを受信，認証に失敗している
    OTHER_FAILURE,          // その他のエラー
  };
  int socket_;
  struct sockaddr_in server_address_;
  char receive_buffer_[kChunkSize];  // 受信したデータをここへためておく
  ssize_t receive_size_ = 0;         // bufferのどこまで埋まっているか

  // NTRIP CasterへのHTTP requestを作成する
  std::string
  createHttpRequest(const std::string& mount_point, const std::string& user_name, const std::string& password);
  MountPointStatus
  checkMountPoint(const std::string& mount_point, const std::string& user_name, const std::string& password);

  // timeout付きでtcp通信で接続する
  static bool connectWithTimeout(const int& fd, const sockaddr* addr, socklen_t len, timeval& timeout);
  // base64にencodeする
  static std::string base64Encode(const std::string& src);
};
}  // namespace ntrip
