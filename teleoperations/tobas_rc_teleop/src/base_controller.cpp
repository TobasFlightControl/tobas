#include "../include/tobas_rc_teleop/base_controller.hpp"

namespace tobas_rc_teleop
{
BaseController::BaseController()
{
  // 不要なrosparamの参照やPubSubの登録を防ぐため，コンストラクタではそれらに関する操作は行わない
}
}  // namespace tobas_rc_teleop
