#pragma once

#include <vector>

#include "../jntarray.hpp"

namespace tobas_kdl
{
void jntarrayKDLToStd(const JntArray& k, std::vector<double>& s);
void jntarrayStdToKDL(const std::vector<double>& s, JntArray& k);
}  // namespace tobas_kdl
