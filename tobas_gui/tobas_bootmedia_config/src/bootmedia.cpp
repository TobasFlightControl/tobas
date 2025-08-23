#include "tobas_bootmedia_config/bootmedia.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
QString BootMedia::string() const
{
  QString res;

  if (!vendor.isEmpty()) {
    res += vendor + " ";
  }

  if (!model.isEmpty()) {
    res += model + " ";
  }

  if (!devnode.isEmpty()) {
    res += "(" + devnode + ")";
  }

  return res.trimmed();
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
