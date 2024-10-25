#pragma once

#include <QToolButton>

namespace gui
{
namespace core
{
class AppButton : public QToolButton
{
  Q_OBJECT

  using self = AppButton;
  using super = QToolButton;

  static constexpr int kIconSize = 40;
  static constexpr int kButtonWidth = 120;

public:
  explicit AppButton(const QString& text, const QString& icon_path);
};
}  // namespace core
}  // namespace gui
