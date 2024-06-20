#pragma once

namespace dsp
{
/* カットオフ周波数から一次遅れフィルタの時定数を計算する． */
inline static double timeConstFromCutoff(const double& cutoff_freq);
}  // namespace dsp
