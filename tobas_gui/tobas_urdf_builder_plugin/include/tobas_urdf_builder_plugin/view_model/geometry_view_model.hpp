#pragma once

#include <memory>

#include <QtCore/QtCore>

#include "./base_view_model.hpp"

using GeometryType = decltype(urdf::Geometry::type);

namespace gui
{
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
  explicit GeometryViewModel(const urdf::GeometrySharedPtr& model);

  void sync() override;

  const QString& name() const;

  GeometryType type() const;
  void type(GeometryType type);
  void type(const QString& type);

  double width() const;
  void width(double width);

  double length() const;
  void length(double length);

  double height() const;
  void height(double height);

  double radius() const;
  void radius(double radius);

  QString filePath() const;
  void filePath(const QString& filepath);

  const urdf::Vector3& scale() const;
  void scale(const urdf::Vector3& scale);

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
}  // namespace gui
