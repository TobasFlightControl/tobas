#pragma once

#include <netinet/in.h>

#include <deque>
#include <string>

namespace ntrip
{
class NtripClient
{
public:
  explicit NtripClient();
  // NTRIP serverへ接続し，mount pointからデータを取得し始める．
  // 通信でブロッキングが発生しているので，この関数を実行するとある程度の時間スレッドが停止する
  bool initialize(
    const char* server_ip,
    const int& server_port,
    const char* mount_point,
    const char* user_name,
    const char* password,
    const double& latitude,
    const double& longitude);
  // mount pointからRTCM3.3 protocolのデータを受信する nonblockingで受信を行う
  void receiveRtcmData();

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
  char receive_buffer_[kChunkSize];  // 受信したデータ用buffer ここではデータ処理は行わずにreceive_dequeへ送りそちらで行う
  std::deque<uint8_t> receive_deque_; // 受信したデータをここへためる 処理したものは前端から削除 受信したものは後端から投入

  // NTRIP CasterへのHTTP requestを作成する
  std::string
  createHttpRequest(const std::string& mount_point, const std::string& user_name, const std::string& password);
  MountPointStatus
  checkMountPoint(const std::string& mount_point, const std::string& user_name, const std::string& password);

  // timeout付きでtcp通信で接続する
  static bool connectWithTimeout(const int& fd, const sockaddr* addr, socklen_t len, timeval& timeout);
  // blockingせず受信する 受信したデータサイズを返す errorのやデータが受信されなかった場合は-1を返す
  static ssize_t nonblockReceive(const int& fd, void* buf, size_t n, int flags);
  // base64にencodeする
  static std::string base64Encode(const std::string& src);
};
}  // namespace ntrip
