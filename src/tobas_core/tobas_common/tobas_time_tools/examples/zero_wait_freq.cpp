// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_time_tools/frequency_measure.hpp>

int main()
{
  tobas::tim::FrequencyMeasure freq_measure;

  while (true) {
    freq_measure.count();
  }
}
