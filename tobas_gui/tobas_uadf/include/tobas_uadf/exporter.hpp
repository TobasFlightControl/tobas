#pragma once

#include <tinyxml2.h>

#include "./model.hpp"

namespace tobas
{
namespace uadf
{
tinyxml2::XMLDocument* exportUADF(const Model& model);
}  // namespace uadf
}  // namespace tobas
