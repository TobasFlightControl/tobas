#include "tobas_gui_common/version.hpp"

#include <format>

#include <tobas_constants/version.hpp>

namespace gui
{
namespace cmn
{
QString version()
{
  QString res = "v%1.%2.%3";
  return res.arg(tobas::version::kMajor).arg(tobas::version::kMinor).arg(tobas::version::kPatch);
}
}  // namespace cmn
}  // namespace gui
