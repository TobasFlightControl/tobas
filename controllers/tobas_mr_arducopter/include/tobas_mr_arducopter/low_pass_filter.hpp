#pragma once

#include "./digital_lpf.hpp"

namespace tobas_mr_arducopter
{
// LPF base class
class LowPassFilter
{
public:
  inline explicit LowPassFilter();
  inline explicit LowPassFilter(double cutoff_freq);
  inline explicit LowPassFilter(double sample_freq, double cutoff_freq);

  // change parameters
  inline void setCutoffFrequency(double cutoff_freq);
  inline void setCutoffFrequency(double sample_freq, double cutoff_freq);

  // return the cutoff frequency
  inline double getCutoffFreq() const;
  inline double apply(double sample, double dt);
  inline double apply(double sample);
  inline const double& get() const;
  inline void reset(double value);
  inline void reset();

private:
  double cutoff_freq_ = 0.;
  DigitalLPF filter_;
};

inline LowPassFilter::LowPassFilter()
{
}

inline LowPassFilter::LowPassFilter(double cutoff_freq) : cutoff_freq_(cutoff_freq)
{
}

inline LowPassFilter::LowPassFilter(double sample_freq, double cutoff_freq)
{
  setCutoffFrequency(sample_freq, cutoff_freq);
}

inline void LowPassFilter::setCutoffFrequency(double cutoff_freq)
{
  cutoff_freq_ = cutoff_freq;
}

inline void LowPassFilter::setCutoffFrequency(double sample_freq, double cutoff_freq)
{
  cutoff_freq_ = cutoff_freq;
  filter_.computeAlpha(sample_freq, cutoff_freq);
}

inline double LowPassFilter::getCutoffFreq() const
{
  return cutoff_freq_;
}

inline double LowPassFilter::apply(double sample, double dt)
{
  return filter_.apply(sample, cutoff_freq_, dt);
}

inline double LowPassFilter::apply(double sample)
{
  return filter_.apply(sample);
}

inline const double& LowPassFilter::get() const
{
  return filter_.get();
}

inline void LowPassFilter::reset(double value)
{
  filter_.reset(value);
}

inline void LowPassFilter::reset()
{
  filter_.reset();
}
}  // namespace tobas_mr_arducopter
