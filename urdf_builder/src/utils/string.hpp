#pragma once

#include <QString>

QString getBaseName(const QString& arg)
{
  return arg.left(arg.lastIndexOf('.'));
}
