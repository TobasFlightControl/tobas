#include "../../include/urdf_builder/view_model/geometry_view_model.hpp"

using namespace std;

static const map<GeometryType, QString> kGeometryTypeToNameMap = {
  { urdf::Geometry::BOX, "Box" },
  { urdf::Geometry::SPHERE, "Sphere" },
  { urdf::Geometry::CYLINDER, "Cylinder" },
  { urdf::Geometry::MESH, "Mesh" },
};

static const map<QString, GeometryType> kGeometryNameToTypeMap = {
  { "Box", urdf::Geometry::BOX },
  { "Sphere", urdf::Geometry::SPHERE },
  { "Cylinder", urdf::Geometry::CYLINDER },
  { "Mesh", urdf::Geometry::MESH },
};

namespace urdf_builder
{
namespace view_model
{
const QString& GeometryViewModel::name() const
{
  return kGeometryTypeToNameMap.at(type_);
}

void GeometryViewModel::type(const QString& type)
{
  type_ = kGeometryNameToTypeMap.at(type);
}

void GeometryViewModel::sync()
{
  switch (type_)
  {
    case GeometryType::BOX:
    {
      auto box = new urdf::Box();
      box->dim.x = length_;
      box->dim.y = width_;
      box->dim.z = height_;
      model_.reset(box);
      break;
    }
    case GeometryType::CYLINDER:
    {
      auto cylinder = new urdf::Cylinder();
      cylinder->length = length_;
      cylinder->radius = radius_;
      model_.reset(cylinder);
      break;
    }
    case GeometryType::SPHERE:
    {
      auto sphere = new urdf::Sphere();
      sphere->radius = radius_;
      model_.reset(sphere);
      break;
    }
    case GeometryType::MESH:
    {
      auto mesh = new urdf::Mesh();
      mesh->filename = filepath_;
      mesh->scale = scale_;
      model_.reset(mesh);
      break;
    }
    default:
    {
      throw runtime_error("Invalid geometry type.");
      break;
    }
  }
}

void GeometryViewModel::load()
{
  radius_ = kDefaultRadius;
  length_ = kDefaultLength;
  width_ = kDefaultWidth;
  height_ = kDefaultHeight;
  filepath_.clear();
  scale_.x = scale_.y = scale_.z = kDefaultScale;

  switch (type_)
  {
    case GeometryType::BOX:
    {
      auto box = urdf::dynamic_pointer_cast<urdf::Box>(model_);
      length_ = box->dim.x;
      width_ = box->dim.y;
      height_ = box->dim.z;
      break;
    }
    case GeometryType::CYLINDER:
    {
      auto cylinder = urdf::dynamic_pointer_cast<urdf::Cylinder>(model_);
      length_ = cylinder->length;
      radius_ = cylinder->radius;
      break;
    }
    case GeometryType::SPHERE:
    {
      auto sphere = urdf::dynamic_pointer_cast<urdf::Sphere>(model_);
      radius_ = sphere->radius;
      break;
    }
    case GeometryType::MESH:
    {
      auto mesh = urdf::dynamic_pointer_cast<urdf::Mesh>(model_);
      filepath_ = mesh->filename;
      scale_ = mesh->scale;
      break;
    }
    default:
    {
      throw runtime_error("Invalid geometry type.");
      break;
    }
  }
}
}  // namespace view_model
}  // namespace urdf_builder
