#pragma once

#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>

namespace gazebo
{
enum sdf_constraint_t
{
  NONE,
  POSITIVE,
  NEGATIVE,
  NON_NEGATIVE,
  NON_POSITIVE,
};

template <typename T>
void checkConstraint(const std::string& name, const T& param, const sdf_constraint_t& constraint)
{
  switch (constraint)
  {
    case NONE:
      break;
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
void getSdfParam(const sdf::ElementPtr& sdf, const std::string& name, T& param)
{
  if (sdf->HasElement(name))
  {
    param = sdf->GetElement(name)->Get<T>();
  }
  else
  {
    gzthrow("Please specify '" << name << "'.");
  }
}

template <typename T>
void getSdfParam(
  const sdf::ElementPtr& sdf,
  const std::string& name,
  T& param,
  const T& default_value,
  const bool& verbose = true)
{
  if (sdf->HasElement(name))
  {
    param = sdf->GetElement(name)->Get<T>();
  }
  else
  {
    if (verbose)
    {
      gzwarn << "SDF parameter '" << name << "' is not specified. The default value '" << default_value << "' is used."
             << std::endl;
    }
    param = default_value;
  }
}

template <typename T>
void getSdfParam(const sdf::ElementPtr& sdf, const std::string& name, T& param, const sdf_constraint_t& constraint)
{
  getSdfParam(sdf, name, param);
  checkConstraint(name, param, constraint);
}

template <typename T>
void getSdfParam(
  const sdf::ElementPtr& sdf,
  const std::string& name,
  T& param,
  const T& default_value,
  const sdf_constraint_t& constraint)
{
  getSdfParam(sdf, name, param, default_value);
  checkConstraint(name, param, constraint);
}

/* SDFからリストを取得． */
template <typename T>
void getSdfParam(const sdf::ElementPtr& sdf, const std::string& name, std::vector<T>& params)
{
  params.clear();

  auto list_elem = sdf->GetElement(name);
  if (list_elem == nullptr)
  {
    gzthrow("Please specify '" << name << "'.");
  }

  auto item_elem = list_elem->GetElement("item");
  while (item_elem)
  {
    const auto value = item_elem->Get<T>();
    params.push_back(value);
    item_elem = item_elem->GetNextElement("item");
  }
}
}  // namespace gazebo
