#include "tobas_setup_assistant/param_getters/pose.hpp"

namespace gui
{
namespace setup_assistant
{
ParamGetterWidget_Pose::ParamGetterWidget_Pose(
  const QString& param_name,
  const QString& description_text,
  int decimals,
  double xyz_min,
  double xyz_max,
  const Eigen::Vector3d& xyz_default,
  const QString& xyz_suffix,
  double rpy_min,
  double rpy_max,
  const Eigen::Vector3d& rpy_default,
  const QString& rpy_suffix)
  : super(param_name, description_text)
{
  // XYZ
  auto cols_xyz = new QHBoxLayout();
  rows_->addLayout(cols_xyz);

  x_ = new DoubleGetter("x", decimals, xyz_min, xyz_max, kXYZSingleStep, xyz_default.x(), xyz_suffix);
  cols_xyz->addWidget(x_);

  y_ = new DoubleGetter("y", decimals, xyz_min, xyz_max, kXYZSingleStep, xyz_default.y(), xyz_suffix);
  cols_xyz->addWidget(y_);

  z_ = new DoubleGetter("z", decimals, xyz_min, xyz_max, kXYZSingleStep, xyz_default.z(), xyz_suffix);
  cols_xyz->addWidget(z_);

  // RPY
  auto cols_rpy = new QHBoxLayout();
  rows_->addLayout(cols_rpy);

  roll_ = new DoubleGetter("roll", decimals, rpy_min, rpy_max, kRPYSingleStep, rpy_default.x(), rpy_suffix);
  cols_rpy->addWidget(roll_);

  pitch_ = new DoubleGetter("pitch", decimals, rpy_min, rpy_max, kRPYSingleStep, rpy_default.y(), rpy_suffix);
  cols_rpy->addWidget(pitch_);

  yaw_ = new DoubleGetter("yaw", decimals, rpy_min, rpy_max, kRPYSingleStep, rpy_default.z(), rpy_suffix);
  cols_rpy->addWidget(yaw_);

  // Connections
  connect(x_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
  connect(y_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
  connect(z_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
  connect(roll_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
  connect(pitch_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
  connect(yaw_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> ParamGetterWidget_Pose::get() const
{
  return { { x(), y(), z() }, { roll(), pitch(), yaw() } };
}

bool ParamGetterWidget_Pose::set(const std::pair<Eigen::Vector3d, Eigen::Vector3d>& src)
{
  const auto& xyz = src.first;
  const auto& rpy = src.second;

  x_->set(xyz.x());
  y_->set(xyz.y());
  z_->set(xyz.z());
  roll_->set(rpy.x());
  pitch_->set(rpy.y());
  yaw_->set(rpy.z());

  return true;
}

double ParamGetterWidget_Pose::x() const
{
  return x_->get();
}

double ParamGetterWidget_Pose::y() const
{
  return y_->get();
}

double ParamGetterWidget_Pose::z() const
{
  return z_->get();
}

double ParamGetterWidget_Pose::roll() const
{
  return roll_->get();
}

double ParamGetterWidget_Pose::pitch() const
{
  return pitch_->get();
}

double ParamGetterWidget_Pose::yaw() const
{
  return yaw_->get();
}

void ParamGetterWidget_Pose::onValueChanged(double)
{
  Q_EMIT valueChanged({ { x(), y(), z() }, { roll(), pitch(), yaw() } });
}
}  // namespace setup_assistant
}  // namespace gui
