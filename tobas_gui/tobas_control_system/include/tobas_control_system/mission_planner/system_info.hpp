#pragma once

#include <QObject>
#include <QString>

namespace gui
{
namespace ctrl
{
/* QMLのコンストラクタ引数． */
class SystemInfo : public QObject
{
  Q_OBJECT

  Q_PROPERTY(QString homeDirectory READ homeDirectory NOTIFY homeDirectoryChanged)

Q_SIGNALS:
  void homeDirectoryChanged();

public:
  explicit SystemInfo(QObject* parent = nullptr);

  QString modelName() const;

  QString homeDirectory() const;

private:
  QString home_dir_;
};
}  // namespace ctrl
}  // namespace gui
