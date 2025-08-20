#pragma once

#include <QMetaType>
#include <QString>

namespace gui
{
namespace bm
{
class BootMedia
{
public:
  QString vendor;
  QString model;
  QString devnode;

  QString string() const;
};
}  // namespace bm
}  // namespace gui

Q_DECLARE_METATYPE(gui::bm::BootMedia);
