#include "tobas_simulation_gui/dynamic_configuration/wind_parameters.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/thread.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

#include "tobas_simulation_gui/dynamic_configuration/constants.hpp"

namespace tobas
{
namespace gui
{
namespace sim
{
WindParamsWidget::WindParamsWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  const auto title = new qt::Label("Wind Parameters", cmn::kLabelPSize, QFont::Bold);

  reset_button_ = new QPushButton("Reset");
  reset_button_->setFixedSize(kHeaderButtonWidth, kHeaderButtonHeight);

  mean_speed_ = new qt::DoubleSliderTextWidget(0., 20., 1);
  direction_ = new qt::IntSliderTextWidget(-180, 180);
  gust_speed_factor_ = new qt::DoubleSliderTextWidget(0., 10., 1);
  gust_duration_ = new qt::DoubleSliderTextWidget(0., 10., 1);
  gust_interval_ = new qt::DoubleSliderTextWidget(0., 30., 1);

  reset();

  // Layout
  const auto header_cols = new QHBoxLayout();
  header_cols->addWidget(title);
  header_cols->addStretch();
  header_cols->addWidget(reset_button_);

  const auto form = new qt::FormLayout();
  form->addVAlignedRow("Mean Speed [m/s]", mean_speed_);
  form->addVAlignedRow("Direction [deg]", direction_);
  form->addVAlignedRow("Gust Speed Factor [-]", gust_speed_factor_);
  form->addVAlignedRow("Gust Duration [s]", gust_duration_);
  form->addVAlignedRow("Gust Interval [s]", gust_interval_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(header_cols);
  rows->addLayout(form);

  setLayout(rows);

  // Connection
  connect(reset_button_, &QPushButton::clicked, this, &self::onResetButtonClicked);
  connect(mean_speed_, &qt::DoubleSliderTextWidget::valueChanged, this, &self::onValueChanged);
  connect(direction_, &qt::IntSliderTextWidget::valueChanged, this, &self::onValueChanged);
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
  // サービスクライアントの準備
  bool success = true;
  QString message;

  qt::startThreadAndWait(
    [&]()
    {
      if (!get_sc_->waitForService()) {
        success = false;
        message = "Failed to connect to \"" + QString(gazebo::kGetWindParamsSrv) + "\" service server.";
        return;
      }
      if (!set_sc_->waitForService()) {
        success = false;
        message = "Failed to connect to \"" + QString(gazebo::kSetWindParamsSrv) + "\" service server.";
        return;
      }
    });

  if (!success) {
    qt::qErrorBox(this, message);
    return false;
  }

  // パラメータの初期値を読み込む
  if (!loadSimParams()) {
    return false;
  }

  // パラメータの初期値を保存
  init_mean_speed_ = getMeanSpeed();
  init_direction_ = getDirection();
  init_gust_speed_factor_ = getGustSpeedFactor();
  init_gust_duration_ = getGustDuration();
  init_gust_interval_ = getGustInterval();

  return true;
}

void WindParamsWidget::reset()
{
  setMeanSpeed(0.);
  setDirection(0.);
  setGustSpeedFactor(0.);
  setGustDuration(0.);
  setGustInterval(0.);
}

double WindParamsWidget::getMeanSpeed() const
{
  return mean_speed_->get();
}

double WindParamsWidget::getDirection() const
{
  return st::deg2rad(direction_->get());
}

double WindParamsWidget::getGustSpeedFactor() const
{
  return gust_speed_factor_->get();
}

double WindParamsWidget::getGustDuration() const
{
  return gust_duration_->get();
}

double WindParamsWidget::getGustInterval() const
{
  return gust_interval_->get();
}

void WindParamsWidget::setMeanSpeed(double value)
{
  QSignalBlocker speed(mean_speed_);
  mean_speed_->set(value);
}

void WindParamsWidget::setDirection(double value_rad)
{
  QSignalBlocker speed(direction_);
  direction_->set(st::rad2deg(value_rad));
}

void WindParamsWidget::setGustSpeedFactor(double value)
{
  QSignalBlocker speed(gust_speed_factor_);
  gust_speed_factor_->set(value);
}

void WindParamsWidget::setGustDuration(double value)
{
  QSignalBlocker speed(gust_duration_);
  gust_duration_->set(value);
}

void WindParamsWidget::setGustInterval(double value)
{
  QSignalBlocker speed(gust_interval_);
  gust_interval_->set(value);
}

bool WindParamsWidget::loadSimParams()
{
  const auto get_req = std::make_shared<GetSrv::Request>();

  if (!get_sc_->call(get_req, kServiceCallTimeout)) {
    qt::qErrorBox(this, "Failed to call \"" + QString(gazebo::kGetWindParamsSrv) + "\" service.");
    return false;
  }

  const auto get_res = get_sc_->getResponse();
  const auto& cur_params = get_res->params;

  setMeanSpeed(cur_params.mean_speed);
  setDirection(cur_params.direction);
  setGustSpeedFactor(cur_params.gust_speed_factor);
  setGustDuration(cur_params.gust_duration);
  setGustInterval(cur_params.gust_interval);

  return true;
}

bool WindParamsWidget::sendGuiParams()
{
  const auto set_req = std::make_shared<SetSrv::Request>();
  set_req->params.mean_speed = getMeanSpeed();
  set_req->params.direction = getDirection();
  set_req->params.gust_speed_factor = getGustSpeedFactor();
  set_req->params.gust_duration = getGustDuration();
  set_req->params.gust_interval = getGustInterval();

  if (!set_sc_->call(set_req, kServiceCallTimeout)) {
    qt::qErrorBox(this, "Failed to call \"" + QString(gazebo::kSetWindParamsSrv) + "\" service.");
    return false;
  }

  const auto set_res = set_sc_->getResponse();
  if (!set_res->success) {
    qt::qErrorBox(this, "Failed to set wind parameters.");
    return false;
  }

  return true;
}

void WindParamsWidget::onResetButtonClicked()
{
  setMeanSpeed(init_mean_speed_);
  setDirection(init_direction_);
  setGustSpeedFactor(init_gust_speed_factor_);
  setGustDuration(init_gust_duration_);
  setGustInterval(init_gust_interval_);

  sendGuiParams();
}

void WindParamsWidget::onValueChanged()
{
  sendGuiParams();
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
