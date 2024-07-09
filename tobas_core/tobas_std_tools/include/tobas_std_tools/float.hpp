#pragma once

namespace tobas_std
{
/* 2つの数値がほとんど等しいときにtrueを返す．GPT4によるとnumpy.isclose()と同じらしい． */
bool isClose(const double& x, const double& y, const double& abs_tol = 1e-8, const double& rel_tol = 1e-5);
}  // namespace tobas_std
