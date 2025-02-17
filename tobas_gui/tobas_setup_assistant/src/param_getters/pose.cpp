#include "tobas_setup_assistant/param_getters/pose.hpp"

namespace gui
{
namespace sa
{
ParamGetterWidget_Pose::ParamGetterWidget_Pose(const QString& param_name, const QString& description_text)
  : super(param_name, description_text)
{
  // XYZ
  const auto cols_xyz = new QHBoxLayout();
  rows_->addLayout(cols_xyz);

  x_ = new DoubleGetter("x");
  x_->setDecimals(kDecimals);
  x_->setSingleStep(kSingleStep);
  x_->setValue(kDefaultValue);
  x_->setSuffix(kXYZSuffix);
  connect(x_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
  cols_xyz->addWidget(x_);

  y_ = new DoubleGetter("y");
  y_->setDecimals(kDecimals);
  y_->setSingleStep(kSingleStep);
  y_->setValue(kDefaultValue);
  y_->setSuffix(kXYZSuffix);
  connect(y_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
  cols_xyz->addWidget(y_);

  z_ = new DoubleGetter("z");
  z_->setDecimals(kDecimals);
  z_->setSingleStep(kSingleStep);
  z_->setValue(kDefaultValue);
  z_->setSuffix(kXYZSuffix);
  connect(z_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
  cols_xyz->addWidget(z_);

  // RPY
  const auto cols_rpy = new QHBoxLayout();
  rows_->addLayout(cols_rpy);

  roll_ = new DoubleGetter("roll");
  roll_->setDecimals(kDecimals);
  roll_->setSingleStep(kSingleStep);
  roll_->setValue(kDefaultValue);
  roll_->setSuffix(kRPYSuffix);
  connect(roll_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
  cols_rpy->addWidget(roll_);

  pitch_ = new DoubleGetter("pitch");
  pitch_->setDecimals(kDecimals);
  pitch_->setSingleStep(kSingleStep);
  pitch_->setValue(kDefaultValue);
  pitch_->setSuffix(kRPYSuffix);
  connect(pitch_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
  cols_rpy->addWidget(pitch_);

  yaw_ = new DoubleGetter("yaw");
  yaw_->setDecimals(kDecimals);
  yaw_->setSingleStep(kSingleStep);
  yaw_->setValue(kDefaultValue);
  yaw_->setSuffix(kRPYSuffix);
  connect(yaw_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
  cols_rpy->addWidget(yaw_);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> ParamGetterWidget_Pose::getValue() const
{
  return { { x(), y(), z() }, { roll(), pitch(), yaw() } };
}

bool ParamGetterWidget_Pose::setValue(const std::pair<Eigen::Vector3d, Eigen::Vector3d>& src)
{
  const auto& xyz = src.first;
  const auto& rpy = src.second;

  x_->setValue(xyz.x());
  y_->setValue(xyz.y());
  z_->setValue(xyz.z());
  roll_->setValue(rpy.x());
  pitch_->setValue(rpy.y());
  yaw_->setValue(rpy.z());

  return true;
}

double ParamGetterWidget_Pose::x() const
{
  return x_->getValue();
}

double ParamGetterWidget_Pose::y() const
{
  return y_->getValue();
}

double ParamGetterWidget_Pose::z() const
{
  return z_->getValue();
}

double ParamGetterWidget_Pose::roll() const
{
  return roll_->getValue();
}

double ParamGetterWidget_Pose::pitch() const
{
  return pitch_->getValue();
}

double ParamGetterWidget_Pose::yaw() const
{
  return yaw_->getValue();
}

void ParamGetterWidget_Pose::onValueChanged(double)
{
  Q_EMIT valueChanged({ { x(), y(), z() }, { roll(), pitch(), yaw() } });
}
}  // namespace sa
}  // namespace gui
