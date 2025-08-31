#pragma once

#include <filesystem>

#include <QWidget>

namespace gui
{
namespace sim
{
class BaseWorldWidget : public QWidget
{
  Q_OBJECT

public:
  virtual std::filesystem::path worldPath() const = 0;
};
}  // namespace sim
}  // namespace gui
