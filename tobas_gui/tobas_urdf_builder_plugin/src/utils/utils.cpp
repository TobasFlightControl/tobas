#include "tobas_urdf_builder_plugin/utils/string.hpp"

namespace gui
{
namespace ub
{
namespace utils
{
QString getBaseName(const QString& arg)
{
  return arg.left(arg.lastIndexOf('.'));
}
}  // namespace utils
}  // namespace ub
}  // namespace gui
