#pragma once

#include <QVBoxLayout>

#include "tobas_qt_tools/widgets/scroll_area.hpp"

namespace qt
{
class ScrollableVBoxLayout : public QVBoxLayout
{
  Q_OBJECT

public:
  explicit ScrollableVBoxLayout(QWidget* parent = nullptr);

  void addWidget(QWidget* widget);
  void addLayout(QLayout* layout);
  void addStretch();

private:
  QVBoxLayout* rows_;
};
}  // namespace qt
