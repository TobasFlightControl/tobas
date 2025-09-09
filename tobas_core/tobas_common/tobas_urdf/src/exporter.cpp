#include "tobas_urdf/exporter.hpp"

#include <urdf_parser/urdf_parser.h>

namespace urdf
{
tinyxml2::XMLDocument* exportUrdf(const urdf::ModelInterface& model)
{
  // FIXME: Avoid deprecated function
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  return urdf::exportURDF(model);
#pragma GCC diagnostic pop
}
}  // namespace urdf
