#pragma once

#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>

namespace gazebo
{
enum Constraint
{
  POSITIVE,
  NEGATIVE,
  NON_NEGATIVE,
  NON_POSITIVE,
};

template <typename T>
void checkConstraint(const std::string& name, const T& param, Constraint constraint)
{
  switch (constraint)
  {
    case POSITIVE:
      if (param <= 0)
      {
        gzthrow(name << " must be positive.");
      }
      break;
    case NEGATIVE:
      if (param >= 0)
      {
        gzthrow(name << " must be negative.");
      }
      break;
    case NON_NEGATIVE:
      if (param < 0)
      {
        gzthrow(name << " must be non-negative.");
      }
      break;
    case NON_POSITIVE:
      if (param > 0)
      {
        gzthrow(name << " must be non-positive.");
      }
      break;
    default:
      gzthrow("Invalid constraint type.");
  }
}

template <typename T>
void getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param)
{
  if (sdf->HasElement(name))
  {
    param = sdf->GetElement(name)->Get<T>();
  }
  else
  {
    gzthrow("Please specify " << name << ".");
  }
}

template <typename T>
void getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value)
{
  if (sdf->HasElement(name))
  {
    param = sdf->GetElement(name)->Get<T>();
  }
  else
  {
    gzwarn << "SDF parameter '" << name << "' is not specified. The default value '"
           << default_value << "' is used." << std::endl;
    param = default_value;
  }
}

template <typename T>
void getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, Constraint constraint)
{
  getSdfParam(sdf, name, param);
  checkConstraint(name, param, constraint);
}

template <typename T>
void getSdfParam(
  sdf::ElementPtr sdf,
  const std::string& name,
  T& param,
  const T& default_value,
  Constraint constraint)
{
  getSdfParam(sdf, name, param, default_value);
  checkConstraint(name, param, constraint);
}
}  // namespace gazebo
