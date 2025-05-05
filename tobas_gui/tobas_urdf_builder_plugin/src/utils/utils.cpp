#include "tobas_urdf_builder_plugin/utils/string.hpp"

namespace gui
{
namespace urdf_builder
{
namespace utils
{
QString getBaseName(const QString& arg)
{
  return arg.left(arg.lastIndexOf('.'));
}
}  // namespace utils
}  // namespace urdf_builder
}  // namespace gui
