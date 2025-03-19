#include <QVBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_gazebo_common/constants.hpp>

#include "tobas_simulation_gui/dynamic_configuration/wind_parameters.hpp"
#include "tobas_simulation_gui/constants.hpp"

namespace gui
{
namespace sim
{
WindParamsWidget::WindParamsWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  const auto title = new qt::Label("Wind Parameters", kLabelPSize, QFont::Bold);

  mean_speed_ = new qt::DoubleSliderTextWidget(0., 20.);
  direction_ = new qt::DoubleSliderTextWidget(-M_PI, M_PI);
  gust_speed_factor_ = new qt::DoubleSliderTextWidget(0., 10.);
  gust_duration_ = new qt::DoubleSliderTextWidget(0., 10.);
  gust_interval_ = new qt::DoubleSliderTextWidget(0., 30.);

  reset();

  // Layout
  const auto form = new qt::FormLayout();
  form->addVAlignedRow("Mean Speed [m/s]", mean_speed_);
  form->addVAlignedRow("Direction [rad]", direction_);
  form->addVAlignedRow("Gust Speed Factor [-]", gust_speed_factor_);
  form->addVAlignedRow("Gust Duration [s]", gust_duration_);
  form->addVAlignedRow("Gust Interval [s]", gust_interval_);

  const auto rows = new QVBoxLayout();
  rows->addWidget(title);
  rows->addLayout(form);

  setLayout(rows);

  // Connection
  connect(mean_speed_, &qt::DoubleSliderTextWidget::valueChanged, this, &self::onValueChanged);
  connect(direction_, &qt::DoubleSliderTextWidget::valueChanged, this, &self::onValueChanged);
  connect(gust_speed_factor_, &qt::DoubleSliderTextWidget::valueChanged, this, &self::onValueChanged);
  connect(gust_duration_, &qt::DoubleSliderTextWidget::valueChanged, this, &self::onValueChanged);
  connect(gust_interval_, &qt::DoubleSliderTextWidget::valueChanged, this, &self::onValueChanged);
}

void WindParamsWidget::updateNamespace(const std::string& ns)
{
  get_sc_ = std::make_shared<ros2::SyncServiceClient<GetSrv>>(node_, path::join(ns, gazebo::kGetWindParamsSrv));
  set_sc_ = std::make_shared<ros2::SyncServiceClient<SetSrv>>(node_, path::join(ns, gazebo::kSetWindParamsSrv));
}

bool WindParamsWidget::start()
{
  if (!get_sc_->waitForService(kWaitForService))
  {
    qt::qErrorBox(this, "Failed to connect to \"" + QString(gazebo::kGetWindParamsSrv) + "\" service server.");
    return false;
  }
  if (!set_sc_->waitForService(kWaitForService))
  {
    qt::qErrorBox(this, "Failed to connect to \"" + QString(gazebo::kSetWindParamsSrv) + "\" service server.");
    return false;
  }

  // パラメータの初期値を設定
  if (!loadCurrentParams())
    return false;

  return true;
}

void WindParamsWidget::reset()
{
  mean_speed_->set(0.);
  direction_->set(0.);
  gust_speed_factor_->set(0.);
  gust_duration_->set(0.);
  gust_interval_->set(0.);
}

bool WindParamsWidget::loadCurrentParams()
{
  const auto get_req = std::make_shared<GetSrv::Request>();

  if (!get_sc_->call(get_req, kServiceCallTimeout))
  {
    qt::qErrorBox(this, "Failed to call \"" + QString(gazebo::kGetWindParamsSrv) + "\" service.");
    return false;
  }

  const auto get_res = get_sc_->getResponse();
  const auto& cur_params = get_res->params;

  mean_speed_->set(cur_params.mean_speed);
  direction_->set(cur_params.direction);
  gust_speed_factor_->set(cur_params.gust_speed_factor);
  gust_duration_->set(cur_params.gust_duration);
  gust_interval_->set(cur_params.gust_interval);

  return true;
}

void WindParamsWidget::onValueChanged()
{
  const auto set_req = std::make_shared<SetSrv::Request>();
  set_req->params.mean_speed = mean_speed_->get();
  set_req->params.direction = direction_->get();
  set_req->params.gust_speed_factor = gust_speed_factor_->get();
  set_req->params.gust_duration = gust_duration_->get();
  set_req->params.gust_interval = gust_interval_->get();

  if (!set_sc_->call(set_req, kServiceCallTimeout))
  {
    qt::qErrorBox(this, "Failed to call \"" + QString(gazebo::kSetWindParamsSrv) + "\" service.");
    return;
  }

  const auto set_res = set_sc_->getResponse();
  if (!set_res->success)
  {
    qt::qErrorBox(this, "Failed to set wind parameters.");
    if (!loadCurrentParams())
      return;
  }
}
}  // namespace sim
}  // namespace gui
