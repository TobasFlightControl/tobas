#pragma once

#include <QMetaType>
#include <QString>

namespace gui
{
namespace bm
{
class Bootmedia
{
public:
  QString vendor;
  QString model;
  QString root_path;

  QString string() const;
};
}  // namespace bm
}  // namespace gui

Q_DECLARE_METATYPE(gui::bm::Bootmedia);
