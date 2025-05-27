#include "tobas_control_system/power_source_viewer/engine_viewer.hpp"

#include <format>

#include <tobas_math/core.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

#define MAX_FUEL_QUANTITY 100.  // TODO: 燃料容量をEngineConfigに含める

namespace gui
{
namespace gcs
{
EngineViewerWidget::EngineViewerWidget(const RosQtBridge& bridge, const tobas::Drone& drone) : drone_(drone)
{
  fuel_quantity_ = new qt::HPositionBarWidget();
  oil_temp_ = new qt::HPositionBarWidget();

  fuel_quantity_->setFixedHeight(kBarHeight);
  oil_temp_->setFixedHeight(kBarHeight);

  // Layout
  const auto form = new qt::FormLayout();
  form->addVAlignedRow(new qt::Label("Fuel Quantity", kLabelPSize), fuel_quantity_);
  form->addVAlignedRow(new qt::Label("Oil Temperature", kLabelPSize), oil_temp_);
  setLayout(form);

  // Connection
  connect(&bridge, &RosQtBridge::engineStateReceived, this, &self::engineStateCb, Qt::QueuedConnection);
}

void EngineViewerWidget::reset()
{
  fuel_quantity_->setUpper(fuel_quantity_->getMinimum());
  fuel_quantity_->setCenterText("");

  oil_temp_->setUpper(oil_temp_->getMinimum());
  oil_temp_->setCenterText("");
}

void EngineViewerWidget::updateInternalDataStructures()
{
  reset();

  if (drone_.prop->type() == tobas::propulsion_system_t::ICE) {
    iprop_ = boost::polymorphic_pointer_downcast<tobas::ICEPropulsionSystemConfig>(drone_.prop);

    fuel_quantity_->setLower(0.);
    fuel_quantity_->setMinimum(0.);
    fuel_quantity_->setMaximum(MAX_FUEL_QUANTITY);

    oil_temp_->setLower(kMinOilTemp);
    oil_temp_->setMinimum(kMinOilTemp);
    oil_temp_->setMaximum(kMaxOilTemp);
  }
  else {
    iprop_.reset();
  }
}

void EngineViewerWidget::updateFuelQuantity(const double& fuel_quantity)
{
  const auto fuel_rate = math::remap(fuel_quantity, 0., MAX_FUEL_QUANTITY, 0., 100.);
  fuel_quantity_->setUpper(fuel_quantity);
  fuel_quantity_->setCenterText(std::format("{:.2f} L ({:.0f} %)", fuel_quantity, fuel_rate).c_str());

  if (fuel_rate > 20.) {
    fuel_quantity_->setFillColor(Qt::green);
  }
  else if (fuel_rate > 10.) {
    fuel_quantity_->setFillColor(Qt::yellow);
  }
  else {
    fuel_quantity_->setFillColor(Qt::red);
  }
}

void EngineViewerWidget::updateOilTemperature(const double& oil_temp)
{
  oil_temp_->setUpper(oil_temp);
  oil_temp_->setCenterText(std::format("{:.1f} ℃", oil_temp).c_str());

  // TODO: 油温の適正値をEngineConfigに含める
  if (oil_temp < 60.) {
    oil_temp_->setFillColor(Qt::blue);
  }
  else if (oil_temp < 100.) {
    oil_temp_->setFillColor(Qt::green);
  }
  else if (oil_temp < 120.) {
    oil_temp_->setFillColor(Qt::yellow);
  }
  else {
    oil_temp_->setFillColor(Qt::red);
  }
}

void EngineViewerWidget::engineStateCb(const tobas_msgs::msg::EngineState::ConstSharedPtr& engine_state)
{
  updateFuelQuantity(engine_state->fuel_quantity);
  updateOilTemperature(engine_state->oil_temperature);
}
}  // namespace gcs
}  // namespace gui
