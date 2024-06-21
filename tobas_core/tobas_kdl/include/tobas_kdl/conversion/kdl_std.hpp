#pragma once

#include <vector>

#include "../jntarray.hpp"

namespace kdl
{
void jntarrayKDLToStd(const JntArray& k, std::vector<double>& s);
void jntarrayStdToKDL(const std::vector<double>& s, JntArray& k);
}  // namespace kdl
