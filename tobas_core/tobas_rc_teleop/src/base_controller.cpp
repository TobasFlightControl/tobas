#include "tobas_rc_teleop/base_controller.hpp"

#include <tobas_path_tools/join.hpp>

using namespace std;

namespace tobas_rc_teleop
{
BaseController::BaseController()
{
  // 不要なrosparamの参照やPubSubの登録を防ぐため，コンストラクタではそれらに関する操作は行わない
}

string BaseController::addMode(const string& text, tobas::FlightMode mode)
{
  return path::join(tobas::textFromEnum(mode), text);
}
}  // namespace tobas_rc_teleop
