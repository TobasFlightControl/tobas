#pragma once

#include <cinttypes>

namespace tobas_std
{
/* 2つの数値がほとんど等しいときにtrueを返す．GPT4によるとnumpy.isclose()と同じらしい． */
bool isClose(const double& x, const double& y, const double& abs_tol = 1e-8, const double& rel_tol = 1e-5);

/* Decode IEEE 754 single precision floating point number. */
float decodeBinary32(uint32_t bin);
}  // namespace tobas_std
