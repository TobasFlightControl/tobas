// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_time_tools/operator.hpp"

std::ostream& operator<<(std::ostream& _os, const tm& _arg)
{
  _os << "Year: " << _arg.tm_year + 1900 << std::endl;  // Years since 1900
  _os << "Month: " << _arg.tm_mon + 1 << std::endl;     // Months since January [0-11]
  _os << "Day: " << _arg.tm_mday << std::endl;
  _os << "Hour: " << _arg.tm_hour << std::endl;
  _os << "Min: " << _arg.tm_min << std::endl;
  _os << "Sec: " << _arg.tm_sec << std::endl;
  return _os;
}

double operator-(tm _lhs, tm _rhs)
{
  const auto time_l = mktime(&_lhs);
  const auto time_r = mktime(&_rhs);
  return std::difftime(time_l, time_r);  // sec
}
