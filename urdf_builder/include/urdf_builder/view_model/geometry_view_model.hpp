#pragma once

#include <memory>
#include <QString>

#include "./base_view_model.hpp"

using GeometryType = decltype(urdf::Geometry::type);

namespace urdf_builder
{
namespace view_model
{
class GeometryViewModel : public BaseViewModel<urdf::Geometry, GeometryViewModel>
{
  static constexpr double kDefaultRadius = 0.;
  static constexpr double kDefaultLength = 0.;
  static constexpr double kDefaultWidth = 0.;
  static constexpr double kDefaultHeight = 0.;
  static constexpr double kDefaultScale = 1.;

public:
  explicit GeometryViewModel(const urdf::GeometrySharedPtr& model)
    : BaseViewModel<urdf::Geometry, GeometryViewModel>(model),
      type_(model_->type),
      radius_(kDefaultRadius),
      length_(kDefaultLength),
      width_(kDefaultWidth),
      height_(kDefaultHeight),
      scale_(kDefaultScale, kDefaultScale, kDefaultScale)
  {
    if (!model)
      model_.reset(new urdf::Sphere());
    load();
  }

  const QString& name() const;

  GeometryType type() const
  {
    return type_;
  }

  void type(GeometryType type)
  {
    type_ = type;
  }

  void type(const QString& type);

  double width() const
  {
    return width_;
  }

  void width(double width)
  {
    width_ = width;
  }

  double length() const
  {
    return length_;
  }

  void length(double length)
  {
    length_ = length;
  }

  double height() const
  {
    return height_;
  }

  void height(double height)
  {
    height_ = height;
  }

  double radius() const
  {
    return radius_;
  }

  void radius(double radius)
  {
    radius_ = radius;
  }

  QString filePath() const
  {
    return QString::fromStdString(filepath_);
  }

  void filePath(const QString& filepath)
  {
    filepath_ = filepath.toStdString();
  }

  const urdf::Vector3& scale() const
  {
    return scale_;
  }

  void scale(const urdf::Vector3& scale)
  {
    scale_ = scale;
  }

  void sync() override;

private:
  GeometryType type_;
  double radius_;
  double length_;
  double width_;
  double height_;
  std::string filepath_;
  urdf::Vector3 scale_;

  void load();
};

using GeometryViewModelPtr = std::shared_ptr<GeometryViewModel>;
}  // namespace view_model
}  // namespace urdf_builder
