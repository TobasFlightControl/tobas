#pragma once

#include <boost/polymorphic_pointer_cast.hpp>

#include "../propulsion_system.hpp"
#include "./engine.hpp"
#include "./ice_rotor.hpp"

namespace tobas
{
class ICEPropulsionSystemConfig : public PropulsionSystemConfig
{
  using self = ICEPropulsionSystemConfig;

  static constexpr char kEngineKey[] = "engine";

public:
  using SharedPtr = std::shared_ptr<ICEPropulsionSystemConfig>;
  using ConstSharedPtr = std::shared_ptr<const ICEPropulsionSystemConfig>;

  EngineConfig engine;

  bool isValid() const override;

  bool load(const YAML::Node& node) override;
  YAML::Node dump() const override;

  propulsion_system_t type() const override;

  double minSpeed(const std::string& link_name) override;
  double maxSpeed(const std::string& link_name) override;

  double minThrust(const std::string& link_name) override;
  double maxThrust(const std::string& link_name) override;

  double thrustFromThrottle(const std::string& link_name, double throttle) override;

  inline ICERotorConfig::SharedPtr getRotor(const std::string& link_name);
  inline ICERotorConfig::ConstSharedPtr getRotor(const std::string& link_name) const;

private:
  std::optional<double> max_engine_speed_;

  /* 平均プロペラピッチ角が固定されているときのエンジンの最大回転数 [rad/s] */
  double maxEngineSpeed();

  /* 平均プロペラピッチ角を固定した上で，エンジンスロットルから定常回転数を求める (memo: 3-29) */
  double computeEngineSpeed(double throttle) const;

  /* ニュートン法ソルバーに渡す関数 (memo: 3-29) */
  double speedFunc(double throttle, double omega) const;
  double speedFuncDeriv(double throttle, double omega) const;

  double calc_phi(double throttle) const;
  double calc_f(double throttle) const;
  double calc_k() const;
};

inline ICERotorConfig::SharedPtr ICEPropulsionSystemConfig::getRotor(const std::string& link_name)
{
  return boost::polymorphic_pointer_downcast<ICERotorConfig>(rotors.at(link_name));
}

inline ICERotorConfig::ConstSharedPtr ICEPropulsionSystemConfig::getRotor(const std::string& link_name) const
{
  return boost::polymorphic_pointer_downcast<ICERotorConfig>(rotors.at(link_name));
}
}  // namespace tobas
