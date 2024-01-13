#include "../include/tobas_rc_teleop/base_controller.hpp"

namespace tobas_rc_teleop
{
BaseController::BaseController(const tobas::Drone& drone) : drone_(drone)
{
  // 不要なrosparamの参照やPubSubの登録を防ぐため，コンストラクタではそれらに関する操作は行わない
}
}  // namespace tobas_rc_teleop
