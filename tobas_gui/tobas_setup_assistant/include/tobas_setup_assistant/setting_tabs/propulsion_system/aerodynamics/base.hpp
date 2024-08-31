#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>
#include <QVBoxLayout>

namespace gui
{
namespace setup_assistant
{
class AerodynamicsWidget_Base : public QWidget
{
  Q_OBJECT

public:
  void initialize();

  virtual const char* name() const = 0;
  virtual const char* description() const = 0;

  virtual void onInit() = 0;

  virtual bool isValid() = 0;
  virtual void copyFrom(const AerodynamicsWidget_Base* src) = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  /* [kg*m/rad^2] */
  virtual double motorConst() const = 0;

  /* [m] */
  virtual double momentConst() const = 0;

  /* [kg/rad] */
  virtual double rotorDragCoef() const = 0;

protected:
  void addWidget(QWidget* widget);
  void addLayout(QLayout* layout);

private:
  QVBoxLayout* header_rows_;
  QVBoxLayout* content_rows_;
};
}  // namespace setup_assistant
}  // namespace gui
