#pragma once

#include "./data.hpp"

namespace tobas
{
namespace cyclonedds
{
bool parseFromText(const std::string& text, Data& dst);
}  // namespace cyclonedds
}  // namespace tobas
