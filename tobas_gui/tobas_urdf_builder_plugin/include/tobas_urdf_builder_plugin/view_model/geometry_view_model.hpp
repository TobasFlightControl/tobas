// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include <QString>

#include "./base_view_model.hpp"

using GeometryType = decltype(::urdf::Geometry::type);

namespace tobas::gui::ub::view_model
{
class GeometryViewModel : public BaseViewModel<::urdf::Geometry, GeometryViewModel>
{
public:
  explicit GeometryViewModel(const ::urdf::GeometrySharedPtr& _model);

  void sync() override;

  const QString& name() const;

  GeometryType type() const;
  void type(GeometryType _type);
  void type(const QString& _type);

  double width() const;
  void width(double _width);

  double length() const;
  void length(double _length);

  double height() const;
  void height(double _height);

  double radius() const;
  void radius(double _radius);

  QString filePath() const;
  void filePath(const QString& _filepath);

  const ::urdf::Vector3& scale() const;
  void scale(const ::urdf::Vector3& _scale);

private:
  GeometryType type_;
  double radius_;
  double length_;
  double width_;
  double height_;
  std::string filepath_;
  ::urdf::Vector3 scale_;

  void load();
};

using GeometryViewModelPtr = std::shared_ptr<GeometryViewModel>;
}  // namespace tobas::gui::ub::view_model
