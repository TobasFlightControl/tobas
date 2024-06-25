#pragma once

namespace dsp
{
/* カットオフ周波数から一次フィルタの時定数を計算する． */
double timeConstFromCutoff(const double& cutoff_freq);
}  // namespace dsp
