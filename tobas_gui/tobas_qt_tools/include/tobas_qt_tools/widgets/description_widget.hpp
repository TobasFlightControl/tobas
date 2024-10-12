#pragma once

#include <QLabel>

namespace qt
{
/* 長めの文章を配置するのに適した設定のQLabel． */
class DescriptionWidget : public QLabel
{
  Q_OBJECT

  using super = QLabel;

public:
  explicit DescriptionWidget(const QString& text, int point_size, QWidget* parent = nullptr);
};
}  // namespace qt
