#include <tobas_wind_model/dryden.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/utils.hpp>

#include <tobas_msgs_adapter/wind.hpp>
#include <tobas_gazebo_msgs/srv/get_wind_params.hpp>
#include <tobas_gazebo_msgs/srv/set_wind_params.hpp>

#include "../include/tobas_gazebo_system_plugins/common/common.hpp"
#include "../include/tobas_gazebo_system_plugins/conversions/gazebo_kdl.hpp"

using namespace std;
namespace cmp = gz::sim::components;

namespace gazebo
{
/**
 * @brief Modeling of Wind Phenomena and Analysis of Their Effects on UAV Trajectory Tracking
 * Performance [Siqueira+, 2017] の4つの風を実装． \n
 *
 * - Constant wind: \n
 * - Turbulance: https://jp.mathworks.com/help/aeroblks/drydenwindturbulencemodeldiscrete.html \n
 * - Wind gust: 1-cosine model (https://aero.w3.kanazawa-u.ac.jp/cgi-bin/wiki.cgi?page=DISTB) \n
 * - Wind shear: // TODO: An overview of various kinds of wind effects on unmanned aerial vehicle \n
 */
class GazeboWindPlugin : public BaseNode,
                         public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPostUpdate
{
  // Default parameters
  static constexpr double kDefaultMeanWindSpeed = 0.;          // [m/s]
  static constexpr double kDefaultConstantWindDirection = 0.;  // [rad]
  static constexpr double kDefaultGustSpeedFactor = 1.;        // [-]
  static constexpr double kDefaultGustDuration = 5.;           // [s]
  static constexpr double kDefaultGustInterval = 10.;          // [s]

  using self = GazeboWindPlugin;
  using GetSrv = tobas_gazebo_msgs::srv::GetWindParams;
  using SetSrv = tobas_gazebo_msgs::srv::SetWindParams;

public:
  explicit GazeboWindPlugin();

  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  enum gust_state_t : uint8_t
  {
    GUST,
    NO_GUST,
  };

  // SDF parameters
  string link_name_;

  const cmp::WorldPose* pose_W_;
  const cmp::WorldLinearVelocity* vel_W_;

  tobas_gazebo_msgs::msg::WindParams params_;
  chrono::steady_clock::duration gust_state_change_time_;
  gust_state_t gust_state_ = NO_GUST;
  double gust_speed_ = 0.;
  tobas::DrydenSimulator dryden_;

  ros2::PublisherPtr<tobas_msgs::Wind> wind_pub_;

  ros2::ServiceServerPtr<GetSrv> get_params_ss_;
  ros2::ServiceServerPtr<SetSrv> set_params_ss_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);

  void getParamsCb(const GetSrv::Request::ConstSharedPtr& req, const GetSrv::Response::SharedPtr& res);
  void setParamsCb(const SetSrv::Request::ConstSharedPtr& req, const SetSrv::Response::SharedPtr& res);
};

GazeboWindPlugin::GazeboWindPlugin()
{
}

void GazeboWindPlugin::Configure(
  const gz::sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_wind_plugin", sdf);
  getSdfParams(sdf);

  // Initialize wind parameters
  params_.mean_speed = kDefaultMeanWindSpeed;
  params_.direction = kDefaultConstantWindDirection;
  params_.gust_speed_factor = kDefaultGustSpeedFactor;
  params_.gust_duration = kDefaultGustDuration;
  params_.gust_interval = kDefaultGustInterval;

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");
  }

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);
  vel_W_ = getComponent<cmp::WorldLinearVelocity>(link, ecm);

  wind_pub_ = createPublisher<tobas_msgs::Wind>(kWindGtTopic);
  get_params_ss_ = createService<GetSrv>(kGetWindParamsSrv, &self::getParamsCb, this);
  set_params_ss_ = createService<SetSrv>(kSetWindParamsSrv, &self::setParamsCb, this);
}

void GazeboWindPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
}

void GazeboWindPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  // 突風
  const auto t_gust = chrono::duration<double>(info.simTime - gust_state_change_time_).count();  // [s]
  switch (gust_state_) {
    case GUST: {
      if (t_gust > params_.gust_duration) {
        gust_state_ = NO_GUST;
        gust_state_change_time_ = info.simTime;
        break;
      }

      const auto max_gust_speed = params_.mean_speed * params_.gust_speed_factor;
      gust_speed_ = 0.5 * max_gust_speed * (1 - cos(2 * M_PI * t_gust / params_.gust_duration));
      break;
    }
    case NO_GUST: {
      if (t_gust > params_.gust_interval) {
        gust_state_ = GUST;
        gust_state_change_time_ = info.simTime;
        break;
      }

      gust_speed_ = 0.;
      break;
    }
    default: {
      TOBAS_EXIT("Invalid gust state: ", static_cast<int>(gust_state_));
    }
  }

  // 定常風 (平均風速 + 突風)
  const auto v_steady_wind = params_.mean_speed + gust_speed_;
  const gz::math::Vector3d steady_W(v_steady_wind * cos(params_.direction), v_steady_wind * sin(params_.direction), 0.);

  // 乱流成分を更新
  const auto rel_wind_speed = (steady_W - vel_W_->Data()).Length();  // 定常風の相対速度
  const auto dt = chrono::duration<double>(info.dt).count();
  dryden_.update(rel_wind_speed, pose_W_->Data().Pos().Z(), dt);
  const gz::math::Vector3d turb_B(dryden_.u(), dryden_.v(), dryden_.w());

  // 全体の風速を計算
  const auto wind_W = steady_W + pose_W_->Data().Rot().RotateVector(turb_B);

  // 風速メッセージを作成
  auto wind_msg = make_unique<tobas_msgs::Wind>();
  wind_msg->header.frame_id = tobas::kWorldFrame;
  vectorGazeboToKDL(wind_W, wind_msg->vel);

  // 風速を発行
  wind_pub_->publish(move(wind_msg));
}

void GazeboWindPlugin::getParamsCb(const GetSrv::Request::ConstSharedPtr&, const GetSrv::Response::SharedPtr& res)
{
  res->params = params_;
}

void GazeboWindPlugin::setParamsCb(const SetSrv::Request::ConstSharedPtr& req, const SetSrv::Response::SharedPtr& res)
{
  res->params = params_;

  // Mean speed
  if (req->params.mean_speed < 0) {
    TOBAS_ERROR("Mean wind speed must be non-negative.");
    res->success = false;
  }
  params_.mean_speed = res->params.mean_speed = req->params.mean_speed;

  // Direction
  params_.direction = res->params.direction = req->params.direction;

  // Gust speed factor
  if (req->params.gust_speed_factor > 0) {
    params_.gust_speed_factor = res->params.gust_speed_factor = req->params.gust_speed_factor;
  }
  else {
    TOBAS_WARN("Gust speed factor remains unchanged.");
  }

  // Gust duration
  if (req->params.gust_duration > 0) {
    params_.gust_duration = res->params.gust_duration = req->params.gust_duration;
  }
  else {
    TOBAS_WARN("Gust duration remains unchanged.");
  }

  // Gust interval
  if (req->params.gust_interval > 0) {
    params_.gust_interval = res->params.gust_interval = req->params.gust_interval;
  }
  else {
    TOBAS_WARN("Gust interval remains unchanged.");
  }

  // Update dryden wind model
  dryden_.setMeanWindSpeed(req->params.mean_speed);

  res->success = true;
  TOBAS_INFO("Wind parameters are updated.");
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboWindPlugin,
  gz::sim::System,
  gazebo::GazeboWindPlugin::ISystemConfigure,
  gazebo::GazeboWindPlugin::ISystemPostUpdate)
