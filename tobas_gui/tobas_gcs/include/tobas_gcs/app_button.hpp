#pragma once

#include <QToolButton>

namespace tobas
{
namespace gui
{
namespace gcs
{
class AppButton : public QToolButton
{
  Q_OBJECT

  using self = AppButton;
  using super = QToolButton;

  static constexpr int kIconHeight = 40;
  static constexpr int kButtonWidth = 120;

public:
  explicit AppButton(const QString& text, const QString& icon_path);
};
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
