#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>
#include <QVBoxLayout>

namespace gui
{
namespace setup_assistant
{
class ElectrodynamicsWidget_Base : public QWidget
{
  Q_OBJECT

public:
  void initialize();

  virtual const char* name() const = 0;
  virtual const char* description() const = 0;

  virtual void onInit() = 0;

  virtual bool isValid() = 0;
  virtual void copyFrom(const ElectrodynamicsWidget_Base* src) = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  /* V = a w + b w^2 (V[V], w[rad/s]) */
  virtual std::pair<double, double> rotSpeedCoefs() const = 0;

protected:
  void addWidget(QWidget* widget);
  void addLayout(QLayout* layout);

private:
  QVBoxLayout* header_rows_;
  QVBoxLayout* content_rows_;
};
}  // namespace setup_assistant
}  // namespace gui
