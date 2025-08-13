#include "tobas_bootmedia_config/bootmedia.hpp"

namespace gui
{
namespace bm
{
QString Bootmedia::string() const
{
  QString res;

  if (!vendor.isEmpty()) {
    res += vendor + " ";
  }

  if (!model.isEmpty()) {
    res += model + " ";
  }

  if (!root_path.isEmpty()) {
    res += "(" + root_path + ")";
  }

  return res.trimmed();
}
}  // namespace bm
}  // namespace gui
