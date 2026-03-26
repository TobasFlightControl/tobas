#include "tobas_qt_tools/layouts/grid_layout.hpp"

#include "tobas_qt_tools/util.hpp"

namespace tobas
{
namespace qt
{
void GridLayout::clear()
{
  clearLayout(this);

  // 各セルの設定をデフォルトに戻す
  for (int row = 0; row < rowCount(); ++row) {
    setRowStretch(row, 0);
    setRowMinimumHeight(row, 0);
  }
  for (int col = 0; col < columnCount(); ++col) {
    setColumnStretch(col, 0);
    setColumnMinimumWidth(col, 0);
  }
}
}  // namespace qt
}  // namespace tobas
