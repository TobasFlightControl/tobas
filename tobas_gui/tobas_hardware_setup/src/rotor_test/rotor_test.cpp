#include <tobas_math/core.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_msgs/srv/set_arm.hpp>

#include "tobas_hardware_setup/rotor_test/rotor_test.hpp"
#include "tobas_hardware_setup/constants.hpp"

namespace gui
{
namespace hardware_setup
{
RotorTestWidget::RotorTestWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone) : node_(node), drone_(drone)
{
  const auto warning =
    new qt::DescriptionWidget("Warning: Ensure that propellers are removed from motors.\n\n", kBodyPSize);
  warning->setStyleSheet("color: red; font-weight: bold;");
  rows_->addWidget(warning);

  const auto instruction = new qt::DescriptionWidget(
    "1. Connect the ESCs to the FC in the correct order.\n\n"
    "2. Press \"Start\" button to enable motors.\n\n"
    "3. For each channel, confirm the followings:\n"
    "   - The motor rotates in the correct direction. If not, swap any two of the three ESC-motor connections.\n"
    "   - The motor does not rotate when the command RPM is 0.\n\n"
    "4. Tune the control gain of each channel to the maximum value at which no vibrations or abnormal noise occur.\n\n"
    "5. Press \"Save\" button to save the control gains.\n\n"
    "6. Press \"Stop\" button to disable motors.\n\n",
    kBodyPSize);
  rows_->addWidget(instruction);

  const auto button_cols = new QHBoxLayout();
  rows_->addLayout(button_cols);

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);
  start_button_->setEnabled(true);
  button_cols->addWidget(start_button_);
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);

  stop_button_ = new QPushButton("Stop");
  stop_button_->setFixedSize(kButtonWidth, kButtonHeight);
  stop_button_->setEnabled(false);
  button_cols->addWidget(stop_button_);
  connect(stop_button_, &QPushButton::clicked, this, &self::onStopButtonClicked);

  save_button_ = new QPushButton("Save");
  save_button_->setFixedSize(kButtonWidth, kButtonHeight);
  save_button_->setEnabled(false);
  button_cols->addWidget(save_button_);
  connect(save_button_, &QPushButton::clicked, this, &self::onSaveButtonClicked);

  button_cols->addStretch();

  const auto rotor_cols = new QHBoxLayout();
  rows_->addLayout(rotor_cols);

  for (size_t ch = 0; ch < kChannelSize; ++ch)
  {
    rotor_widgets_.at(ch) = new RotorWidget();
    rotor_cols->addWidget(rotor_widgets_.at(ch));
    connect(
      rotor_widgets_.at(ch), &RotorWidget::targetRPMChanged,
      std::bind(&self::onTargetRPMChanged, this, std::placeholders::_1, ch));
    connect(
      rotor_widgets_.at(ch), &RotorWidget::gainChanged,
      std::bind(&self::onGainChanged, this, std::placeholders::_1, ch));
  }

  connect(&update_timer_, &QTimer::timeout, this, &self::onUpdateTimerTimeout);

  setEnabled(false);
}

const char* RotorTestWidget::name() const
{
  return "Rotor Test";
}

const char* RotorTestWidget::title() const
{
  return "Test Rotors";
}

void RotorTestWidget::updateInternalDataStructures()
{
  reset();

  // モータとして登録されているチャンネルの設定
  QSet<size_t> rotor_channels;
  for (const auto& [channel, rotor] : drone_.rotors)
  {
    rotor_channels.insert(channel);

    const auto text = "CH" + QString::number(channel) + ": " + QString::fromStdString(rotor.link_name);
    rotor_widgets_.at(channel)->setText(text);

    // XXX: 最大値がtickmarkStepSizeの整数倍じゃないとステップサイズが正しく反映されない
    const auto max_rpm = tobas_std::rps2rpm(rotor.max_rot_speed);
    rotor_widgets_.at(channel)->setMaximumRPM(max_rpm);
  }

  // モータとして登録されていないチャンネルの設定
  for (size_t ch = 0; ch < kChannelSize; ++ch)
  {
    if (rotor_channels.contains(ch))
      continue;

    const auto text = "CH" + QString::number(ch) + ": unregistered";
    rotor_widgets_.at(ch)->setText(text);
  }

  tar_speeds_pub_ = ros2::createPublisher<tobas_msgs::msg::RotorSpeedArray>(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kRotorSpeedsCmdTopic));
  cur_states_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kRotorStatesTopic), &self::currentStatesCb, this);
  arming_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCb, this);

  set_arm_sc_ = std::make_shared<ros2::SyncServiceClient<tobas_msgs::srv::SetArm>>(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kSetArmSrv));
  get_gains_sc_ = std::make_shared<ros2::SyncServiceClient<tobas_msgs::srv::GetRotorControlGains>>(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kGetRotorControlGainsSrv));
  set_gains_sc_ = std::make_shared<ros2::SyncServiceClient<tobas_msgs::srv::SetRotorControlGains>>(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kSetRotorControlGainsSrv));
  save_gains_sc_ = std::make_shared<ros2::SyncServiceClient<std_srvs::srv::Trigger>>(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kSaveRotorControlGainsSrv));

  setEnabled(true);
}

void RotorTestWidget::reset()
{
  // モータウィジェットを無効化
  for (auto& rotor_widget : rotor_widgets_)
  {
    rotor_widget->reset();
    rotor_widget->setEnabled(false);
  }

  // タイマーを停止
  update_timer_.stop();

  start_button_->setEnabled(true);
  stop_button_->setEnabled(false);
  save_button_->setEnabled(false);

  cur_states_ = nullptr;
  arming_ = nullptr;
}

void RotorTestWidget::publishTargetSppeds()
{
  if (tar_speeds_pub_ == nullptr)
    return;

  auto tar_speeds = std::make_unique<tobas_msgs::msg::RotorSpeedArray>();
  tar_speeds->header.stamp = node_->get_clock()->now();

  for (const auto& [channel, _] : drone_.rotors)
  {
    tar_speeds->speeds.emplace_back();
    tar_speeds->speeds.back().channel = channel;
    tar_speeds->speeds.back().speed = tobas_std::rpm2rps(rotor_widgets_.at(channel)->getTargetRPM());
  }

  tar_speeds_pub_->publish(std::move(tar_speeds));
}

void RotorTestWidget::updateCurrentSpeeds()
{
  if (cur_states_ == nullptr)
    return;

  for (const auto& state : cur_states_->states)
  {
    if (state.status == tobas_msgs::msg::RotorState::NO_COMMUNICATION)
      continue;
    if (state.channel >= rotor_widgets_.size())
      continue;

    rotor_widgets_.at(state.channel)->setCurrentRPM(tobas_std::rps2rpm(state.speed));
  }
}

bool RotorTestWidget::loadCurrentGains()
{
  const auto req = std::make_shared<tobas_msgs::srv::GetRotorControlGains::Request>();
  if (!get_gains_sc_->call(req, kWaitForService))
  {
    qt::qErrorBox(this, "Failed to connect to the rotor controller.");
    return false;
  }

  const auto res = get_gains_sc_->getResponse();
  const auto& gains = res->gains;
  for (const auto& [channel, _] : drone_.rotors)
  {
    if (channel >= gains.size())
    {
      qt::qErrorBox(this, "Rotor channel " + QString::number(channel) + " is out of range.");
      return false;
    }
    rotor_widgets_.at(channel)->setGain(gains.at(channel));
  }

  return true;
}

bool RotorTestWidget::armRotors(bool arming)
{
  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = arming;
  req->ignore_prearm_check = true;
  if (!set_arm_sc_->call(req, kWaitForService))
  {
    qt::qErrorBox(this, "Failed to connect to the rotor controller.");
    return false;
  }

  const auto res = set_arm_sc_->getResponse();
  if (!res->success)
  {
    qt::qErrorBox(this, "Arming service failed: " + QString::fromStdString(res->message));
    return false;
  }

  return true;
}

void RotorTestWidget::currentStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& cur_states)
{
  cur_states_ = cur_states;
}

void RotorTestWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void RotorTestWidget::onStartButtonClicked()
{
  // アームされていないことを確認
  if (arming_ == nullptr)
  {
    qt::qWarnBox(this, "This operation cannot be performed because the arming status is not received yet.");
    return;
  }
  if (arming_->data)
  {
    qt::qWarnBox(this, "This operation cannot be performed because the rotors are already armed.");
    return;
  }

  // 現在のゲインを反映
  if (!loadCurrentGains())
    return;

  // モータを起動
  if (!armRotors(true))
    return;

  // モータウィジェットを有効化
  for (const auto& [channel, _] : drone_.rotors)
    rotor_widgets_.at(channel)->setEnabled(true);

  // 一定周期でコマンドの発行と状態の更新
  update_timer_.start(kUpdatePeriod);

  start_button_->setEnabled(false);
  stop_button_->setEnabled(true);
  save_button_->setEnabled(true);

  qt::qInfoBox(this, "Rotor test is started.");
}

void RotorTestWidget::onStopButtonClicked()
{
  // モータを停止
  if (!armRotors(false))
    return;

  // ウィジェットを初期化
  reset();

  qt::qInfoBox(this, "Rotor test is finished.");
}

void RotorTestWidget::onSaveButtonClicked()
{
  const auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

  if (!save_gains_sc_->call(req, kWaitForService))
  {
    qt::qErrorBox(this, "Failed to connect to the rotor controller.");
    return;
  }

  const auto res = set_gains_sc_->getResponse();
  if (!res->success)
  {
    qt::qErrorBox(this, "Failed to save control gains: " + QString::fromStdString(res->message));
    return;
  }

  qt::qInfoBox(this, "Control gains are saved successfully.");
}

void RotorTestWidget::onTargetRPMChanged(int, size_t)
{
  publishTargetSppeds();
}

void RotorTestWidget::onGainChanged(int gain, size_t ch)
{
  const auto req = std::make_shared<tobas_msgs::srv::SetRotorControlGains::Request>();
  req->gains.emplace_back();
  req->gains.back().channel = ch;
  req->gains.back().gain = gain;

  if (!set_gains_sc_->call(req, kWaitForService))
  {
    qt::qErrorBox(this, "Failed to connect to the rotor controller.");
    return;
  }

  const auto res = set_gains_sc_->getResponse();
  if (!res->success)
  {
    qt::qErrorBox(this, "Failed to set control gains: " + QString::fromStdString(res->message));
    return;
  }
}

void RotorTestWidget::onUpdateTimerTimeout()
{
  // モータが停止しないよう，スライダーに変化がなくても一定周期でコマンドを発行する．
  publishTargetSppeds();

  // ROSとQtのスレッドの競合を防ぐため，GUI関連の処理は必ずQtスレッドで行う．
  updateCurrentSpeeds();
}
}  // namespace hardware_setup
}  // namespace gui
