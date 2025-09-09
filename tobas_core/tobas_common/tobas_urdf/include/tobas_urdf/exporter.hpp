#pragma once

#include <tinyxml2.h>
#include <urdf_model/model.h>

namespace urdf
{
tinyxml2::XMLDocument* exportUrdf(const urdf::ModelInterface& model);
}  // namespace urdf
