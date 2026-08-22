#include <tobas_setup_assistant/setting_tabs/fixed_wing/helper/helper.hpp>

#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/util.hpp>

#include <tobas_setup_assistant/setting_tabs/fixed_wing/helper/calculator.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
namespace hp
{
HelperWidget::HelperWidget(const uadf::Model& uadf, const kdl::Tree& tree, VehicleParametersWidget* vehicle)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  // Aerodynamic Coefficients
  rows->addWidget(new qt::Label(kWingsLabel, cmn::kTitlePSize));
  wings_ = new WingsWidget();
  rows->addWidget(wings_);

  // Control Surfaces
  rows->addWidget(new qt::Label(kControlSurfacesLabel, cmn::kTitlePSize));
  control_surfaces_ = new ControlSurfacesWidget(uadf, wings_);
  rows->addWidget(control_surfaces_);

  // calculate button
  calculator_ = new Calculator(tree, vehicle, wings_, control_surfaces_);
  qt::addWidgetCenter(calculator_, rows);
}

const char* HelperWidget::name() const
{
  return "Helper";
}

void HelperWidget::updateInternalDataStructures()
{
  wings_->updateInternalDataStructures();
  control_surfaces_->updateInternalDataStructures();
  calculator_->updateInternalDataStructures();
}

void HelperWidget::setToDefaults()
{
  wings_->setToDefaults();
  control_surfaces_->setToDefaults();
}

bool HelperWidget::isValid()
{
  return wings_->isValid() && control_surfaces_->isValid();
}

YAML::Node HelperWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  return node;
}

void HelperWidget::load(const YAML::Node&)
{
}
}  // namespace hp
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas

