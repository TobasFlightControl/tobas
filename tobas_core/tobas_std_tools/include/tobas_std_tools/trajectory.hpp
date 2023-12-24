#pragma once

namespace tobas_std
{
/* 軌道生成器の基底クラス */
class BaseTrajectory
{
public:
  virtual void get(const double& t, double& p, double& v, double& a) = 0;
  virtual double duration() = 0;
};

/* 3次多項式軌道生成 (ロボティクス, p.192) */
class CubicSpline : public BaseTrajectory
{
public:
  explicit CubicSpline(const double& p0, const double& pf, const double& T);

  void get(const double& t, double& p, double& v, double& a) override;
  double duration() override;

private:
  double T_;
  double a0_;
  double a1_;
  double a2_;
  double a3_;
};
}  // namespace tobas_std
