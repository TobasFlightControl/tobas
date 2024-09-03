#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/observer/observer.hpp"
#include "tobas_setup_assistant/setting_tabs/observer/eskf.hpp"
#include "tobas_setup_assistant/setting_tabs/observer/custom.hpp"

namespace gui
{
namespace setup_assistant
{
ObserverWidget::ObserverWidget(const IMUWidget* imu, const BarometerWidget* baro, const GPSWidget* gps)
  : imu_(imu), baro_(baro), gps_(gps)
{
}

const char* ObserverWidget::name() const
{
  return "Observer";
}

const char* ObserverWidget::title() const
{
  return "Setup Observer";
}

const char* ObserverWidget::description() const
{
  return "Configure the state estimator by selecting one method and setting its parameters. "
         "You can tune the parameters later, so it's fine to leave them at their default values if preferred.";
}

void ObserverWidget::onInit()
{
  type_ = new qt::ComboBox();
  observers_ = new qt::StackedWidget();
  description_ = new qt::DescriptionWidget("", kBodyPSize);

  addWidget(type_);
  addWidget(description_);
  addWidget(observers_);

  observers_->addWidget(new ErrorStateKalmanFilterWidget(imu_, baro_, gps_));
  observers_->addWidget(new CustomObserverWidget());

  for (int i = 0; i < observers_->count(); ++i)
  {
    const auto observer = qobject_cast<BaseObserverWidget*>(observers_->widget(i));
    type_->addItem(observer->name());
  }

  connect(type_, QOverload<int>::of(&qt::ComboBox::currentIndexChanged), this, &self::setCurrentObserver);
  setCurrentObserver(0);
}

void ObserverWidget::onOpened()
{
  return;
}

void ObserverWidget::updateInternalDataStructures()
{
  return;
}

bool ObserverWidget::isValid()
{
  if (!selected()->isValid())
    return false;

  return true;
}

YAML::Node ObserverWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[kTypeKey] = type_->currentText();

  for (int i = 0; i < observers_->count(); ++i)
  {
    const auto observer = qobject_cast<BaseObserverWidget*>(observers_->widget(i));
    node[observer->name()] = observer->dump();
  }

  return node;
}

void ObserverWidget::load(const YAML::Node& node)
{
  type_->setCurrentText(node[kTypeKey].as<QString>());

  for (int i = 0; i < observers_->count(); ++i)
  {
    const auto observer = qobject_cast<BaseObserverWidget*>(observers_->widget(i));
    observer->load(node[observer->name()]);
  }
}

const char* ObserverWidget::observerPackage() const
{
  return selected()->observerPackage();
}

YAML::Node ObserverWidget::staticParams() const
{
  return selected()->staticParams();
}

void ObserverWidget::setCurrentObserver(int index)
{
  observers_->setCurrentIndex(index);
  description_->setText(selected()->description());
}

BaseObserverWidget* ObserverWidget::selected()
{
  return qobject_cast<BaseObserverWidget*>(observers_->currentWidget());
}

const BaseObserverWidget* ObserverWidget::selected() const
{
  return qobject_cast<const BaseObserverWidget*>(observers_->currentWidget());
}
}  // namespace setup_assistant
}  // namespace gui
