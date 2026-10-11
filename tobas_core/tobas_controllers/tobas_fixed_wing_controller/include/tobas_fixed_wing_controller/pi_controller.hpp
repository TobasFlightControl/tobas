#pragma once

namespace tobas
{
namespace fixed_wing
{
// TODO: 別のライブラリに移す
// 速度型PI制御器 アンチワインドアップする
class PIController
{
public:
  explicit PIController();
  explicit PIController(const double& p, const double& i);
  void setGains(const double& p, const double& i);
  void setMaxMin(const double& max, const double& min);
  void update(const double& u, const double& dt);
  double getValue();

private:
  double p_;
  double i_;

  double input_ = 0.0;
  double last_u_ = 0.0;
  double max_ = 0.0;
  double min_ = 0.0;
};
} // namespace fixed_wing
} // namespace tobas
