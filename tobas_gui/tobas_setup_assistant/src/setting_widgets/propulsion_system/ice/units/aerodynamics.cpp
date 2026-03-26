#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/aerodynamics.hpp"

#include <eigen3/Eigen/SVD>

#include <tobas_eigen_tools/core.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/constants.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/blade_theory.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
AerodynamicsWidget::AerodynamicsWidget(rclcpp::Node::SharedPtr node, const PropellerWidget* propeller)
  : propeller_(propeller)
{
  data_ = new ParamGetterWidget_DoubleTable(
    node, "Propeller Single Test Data", "Select Test Data", { "RPM", "Pitch [deg]", "Thrust [N]", "Torque [Nm]" });
  data_->setDecimals({ 0, 2, 6, 6 });
  data_->setMinimum({ 1, -90, 1e-6, 1e-6 });
  data_->setMaximum({ INFINITY, 90, INFINITY, INFINITY });
  data_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  data_->table()->setColumnsWidth(kDataTableColWidth);

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(data_);
  setLayout(rows);
}

const char* AerodynamicsWidget::name() const
{
  return "Aerodynamics";
}

bool AerodynamicsWidget::isValid()
{
  if (data_->count() == 0) {
    tobas::qt::qWarnBox(this, "Propeller test data is blank.");
    return false;
  }

  if (!motorConst().isValid()) {
    tobas::qt::qWarnBox(this, "Failed to estimate the motor constant of the variable pitch propeller.");
    return false;
  }

  if (!momentConst().isValid()) {
    tobas::qt::qWarnBox(this, "Failed to estimate the moment constant of the variable pitch propeller.");
    return false;
  }

  if (!dragConst().isValid()) {
    tobas::qt::qWarnBox(this, "Failed to estimate the drag constant of the variable pitch propeller.");
    return false;
  }

  return true;
}

void AerodynamicsWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = tobas::qt::qConstPointerCast<AerodynamicsWidget>(src);
  data_->setValue(derived->data_->getValue());
}

YAML::Node AerodynamicsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[data_->name()] = data_->getValue();

  return node;
}

void AerodynamicsWidget::load(const YAML::Node& node)
{
  data_->setValue(node[data_->name()].as<Eigen::MatrixXd>());
}

tobas::VppMotorConstant AerodynamicsWidget::motorConst() const
{
  const auto [speed, pitch, thrust, _] = getData();
  const auto num_data = speed.size();

  Eigen::MatrixX2d Left(num_data, 2);
  const auto speed_sqr = speed.cwiseAbs2().eval();
  Left.col(0) = speed_sqr;
  Left.col(1) = pitch.cwiseProduct(speed_sqr);

  const auto sol = Left.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(thrust).eval();
  const auto c0 = sol(0);
  const auto c1 = sol(1);

  return tobas::VppMotorConstant(c0, c1);
}

tobas::VppMomentConstant AerodynamicsWidget::momentConst() const
{
  const auto [c0, c1] = motorConst();
  const auto phi0 = -c0 / c1;

  const auto [_, pitch, thrust, torque] = getData();
  const auto num_data = pitch.size();

  Eigen::MatrixX3d Left(num_data, 3);
  const auto phi = (pitch.array() - phi0).matrix().eval();
  Left.col(0) = phi;
  Left.col(1).setOnes();
  Left.col(2) = phi.cwiseInverse();

  const auto cm = torque.cwiseProduct(thrust.cwiseInverse());

  const auto sol = Left.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(cm).eval();
  const auto a = sol(0);
  const auto b = sol(1);
  const auto c = sol(2);

  return tobas::VppMomentConstant(a, b, c, phi0);
}

tobas::VppDragConstant AerodynamicsWidget::dragConst() const
{
  const BladeTheory blade(
    propeller_->numBlades(), propeller_->radius(), propeller_->meanChord(), propeller_->pitchAngleNeutoral());
  return blade.dragConst();
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> AerodynamicsWidget::getData() const
{
  std::vector<double> speeds, pitches, thrusts, torques;

  const auto num_data = data_->count();
  const auto data_mat = data_->getValue();

  const auto pitch_limit = propeller_->pitchAngleLimit();

  for (int i = 0; i < num_data; ++i) {
    const auto speed = st::rpm2rps(data_mat(i, 0));  // [rad/s]
    const auto pitch = st::deg2rad(data_mat(i, 1));  // [rad]
    const auto thrust = data_mat(i, 2);              // [N]
    const auto torque = data_mat(i, 3);              // [Nm]

    if (pitch_limit.inRange(pitch, 1e-3)) {
      speeds.push_back(speed);
      pitches.push_back(pitch);
      thrusts.push_back(thrust);
      torques.push_back(torque);
    }
  }

  return { eigen::fromStdVector(speeds),
           eigen::fromStdVector(pitches),
           eigen::fromStdVector(thrusts),
           eigen::fromStdVector(torques) };
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
