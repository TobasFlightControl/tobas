#pragma once

namespace tobas_mr_arducopter
{
// DigitalLPF implements the filter math
class DigitalLPF
{
public:
  explicit DigitalLPF();

  // add a new raw value to the filter, retrieve the filtered result
  double apply(const double& sample, double cutoff_freq, double dt);
  double apply(const double& sample);

  void computeAlpha(double sample_freq, double cutoff_freq);

  // get latest filtered value from filter (equal to the value returned by latest call to apply
  // method)
  inline const double& get() const;
  inline void reset();
  void reset(double value);

private:
  double output_ = 0.;
  double alpha_ = 1.;
  bool initialized_;
};

inline const double& DigitalLPF::get() const
{
  return output_;
}

inline void DigitalLPF::reset()
{
  initialized_ = false;
}
}  // namespace tobas_mr_arducopter
