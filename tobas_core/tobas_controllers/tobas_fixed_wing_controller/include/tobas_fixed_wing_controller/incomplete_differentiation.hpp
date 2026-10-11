#pragma once

namespace tobas
{
namespace fixed_wing
{
// TODO: 別のライブラリにうつす
class IncompleteDifferentiation
{
private:
  /* data */
public:
  explicit IncompleteDifferentiation();
  explicit IncompleteDifferentiation(const double& cutoff_frequency);
  void setCutoffFrequency(const double& freq);
  void update(const double& u, const double& dt);
  double getValue();

private:
  double tau_ = 0.0;
  double theta_ = 0.0;
  double u_last_ = 0.0;
};

}  // namespace fixed_wing
}  // namespace tobas
