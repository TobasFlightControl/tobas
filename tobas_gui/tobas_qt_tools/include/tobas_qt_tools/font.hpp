#pragma once

#include <QFont>

namespace tobas
{
namespace qt
{
/* デフォルトでデフォルトの書式を使用するQFont． */
class DefaultFont : public QFont
{
public:
  explicit DefaultFont(int point_size = -1, int weight = -1, bool italic = false);
};
}  // namespace qt
}  // namespace tobas
