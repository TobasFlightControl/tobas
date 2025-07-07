#pragma once

#include "./model.hpp"

namespace uadf
{
bool parseFromText(const std::string& uadf_text, Model& uadf_model);
}  // namespace uadf
