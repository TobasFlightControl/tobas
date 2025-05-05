#pragma once

#include <yaml-cpp/yaml.h>
#include <QVBoxLayout>
#include <QWidget>

#include <tobas_qt_tools/widgets/description_widget.hpp>

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class EngineDynamicsWidget_Base : public QWidget
{
  Q_OBJECT

public:
  explicit EngineDynamicsWidget_Base();

  virtual const char* name() const = 0;
  virtual const char* description() const = 0;

  virtual bool isValid() = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  /* [Nm/(rad/s)] */
  virtual double torqueConstant() const = 0;

  /* [Nm] */
  virtual double dynamicFrictionTorque() const = 0;

protected:
  void addWidget(QWidget* widget);
  void addLayout(QLayout* layout);

private:
  qt::DescriptionWidget* description_;

  QVBoxLayout* header_rows_;
  QVBoxLayout* content_rows_;

private Q_SLOTS:
  void initialize();
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
