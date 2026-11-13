// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf/exporter.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>

#include <urdf_parser/urdf_parser.h>

namespace tobas::urdf
{
namespace
{
using Element = tinyxml2::XMLElement;

void setScalar(Element* _element, const char* _attribute, double _value)
{
  _element->SetAttribute(_attribute, urdf_export_helpers::values2str(_value).c_str());
}

void setVector(Element* _element, const char* _attribute, const ::urdf::Vector3& _value)
{
  _element->SetAttribute(_attribute, urdf_export_helpers::values2str(_value).c_str());
}

void exportOrigin(Element* _parent, const ::urdf::Pose& _pose)
{
  const auto origin = _parent->InsertNewChildElement("origin");
  setVector(origin, "xyz", _pose.position);
  origin->SetAttribute("rpy", urdf_export_helpers::values2str(_pose.rotation).c_str());
}

void exportMaterial(Element* _parent, const ::urdf::Material& _material)
{
  const auto material = _parent->InsertNewChildElement("material");
  material->SetAttribute("name", _material.name.c_str());
  const auto color = material->InsertNewChildElement("color");
  color->SetAttribute("rgba", urdf_export_helpers::values2str(_material.color).c_str());
  if (!_material.texture_filename.empty()) {
    material->InsertNewChildElement("texture")->SetAttribute("filename", _material.texture_filename.c_str());
  }
}

void exportGeometry(Element* _parent, const ::urdf::Geometry& _geometry)
{
  const auto geometry = _parent->InsertNewChildElement("geometry");
  switch (_geometry.type) {
    case ::urdf::Geometry::SPHERE: {
      const auto& sphere = static_cast<const ::urdf::Sphere&>(_geometry);
      setScalar(geometry->InsertNewChildElement("sphere"), "radius", sphere.radius);
      break;
    }
    case ::urdf::Geometry::BOX: {
      const auto& box = static_cast<const ::urdf::Box&>(_geometry);
      setVector(geometry->InsertNewChildElement("box"), "size", box.dim);
      break;
    }
    case ::urdf::Geometry::CYLINDER: {
      const auto& cylinder = static_cast<const ::urdf::Cylinder&>(_geometry);
      const auto element = geometry->InsertNewChildElement("cylinder");
      setScalar(element, "radius", cylinder.radius);
      setScalar(element, "length", cylinder.length);
      break;
    }
    case ::urdf::Geometry::CAPSULE: {
      const auto& capsule = static_cast<const ::urdf::Capsule&>(_geometry);
      const auto element = geometry->InsertNewChildElement("capsule");
      setScalar(element, "radius", capsule.radius);
      setScalar(element, "length", capsule.length);
      break;
    }
    case ::urdf::Geometry::MESH: {
      const auto& mesh = static_cast<const ::urdf::Mesh&>(_geometry);
      const auto element = geometry->InsertNewChildElement("mesh");
      element->SetAttribute("filename", mesh.filename.c_str());
      setVector(element, "scale", mesh.scale);
      break;
    }
    default:
      throw std::invalid_argument("Cannot export an unknown URDF geometry type.");
  }
}

void exportVisual(Element* _parent, const ::urdf::Visual& _visual)
{
  const auto visual = _parent->InsertNewChildElement("visual");
  if (!_visual.name.empty()) {
    visual->SetAttribute("name", _visual.name.c_str());
  }
  exportOrigin(visual, _visual.origin);
  if (_visual.geometry) {
    exportGeometry(visual, *_visual.geometry);
  }
  if (_visual.material) {
    exportMaterial(visual, *_visual.material);
  }
  else if (!_visual.material_name.empty()) {
    visual->InsertNewChildElement("material")->SetAttribute("name", _visual.material_name.c_str());
  }
}

void exportCollision(Element* _parent, const ::urdf::Collision& _collision)
{
  const auto collision = _parent->InsertNewChildElement("collision");
  if (!_collision.name.empty()) {
    collision->SetAttribute("name", _collision.name.c_str());
  }
  exportOrigin(collision, _collision.origin);
  if (_collision.geometry) {
    exportGeometry(collision, *_collision.geometry);
  }
}

void exportLink(Element* _parent, const ::urdf::Link& _link)
{
  const auto link = _parent->InsertNewChildElement("link");
  link->SetAttribute("name", _link.name.c_str());
  if (_link.inertial) {
    const auto& data = *_link.inertial;
    const auto inertial = link->InsertNewChildElement("inertial");
    exportOrigin(inertial, data.origin);
    setScalar(inertial->InsertNewChildElement("mass"), "value", data.mass);
    const auto inertia = inertial->InsertNewChildElement("inertia");
    setScalar(inertia, "ixx", data.ixx);
    setScalar(inertia, "ixy", data.ixy);
    setScalar(inertia, "ixz", data.ixz);
    setScalar(inertia, "iyy", data.iyy);
    setScalar(inertia, "iyz", data.iyz);
    setScalar(inertia, "izz", data.izz);
  }
  for (const auto& visual : _link.visual_array) {
    exportVisual(link, *visual);
  }
  if (_link.visual_array.empty() && _link.visual) {
    exportVisual(link, *_link.visual);
  }
  for (const auto& collision : _link.collision_array) {
    exportCollision(link, *collision);
  }
  if (_link.collision_array.empty() && _link.collision) {
    exportCollision(link, *_link.collision);
  }
}

void setFiniteAttribute(Element* _element, const char* _name, double _value)
{
  // URDF 1.2 represents unbounded limits by omitting the attribute.
  if (std::isfinite(_value)) {
    setScalar(_element, _name, _value);
  }
}

void exportJoint(Element* _parent, const ::urdf::Joint& _joint)
{
  const auto joint = _parent->InsertNewChildElement("joint");
  joint->SetAttribute("name", _joint.name.c_str());
  switch (_joint.type) {
    case ::urdf::Joint::REVOLUTE:
      joint->SetAttribute("type", "revolute");
      break;
    case ::urdf::Joint::CONTINUOUS:
      joint->SetAttribute("type", "continuous");
      break;
    case ::urdf::Joint::PRISMATIC:
      joint->SetAttribute("type", "prismatic");
      break;
    case ::urdf::Joint::FLOATING:
      joint->SetAttribute("type", "floating");
      break;
    case ::urdf::Joint::PLANAR:
      joint->SetAttribute("type", "planar");
      break;
    case ::urdf::Joint::FIXED:
      joint->SetAttribute("type", "fixed");
      break;
    case ::urdf::Joint::UNKNOWN:
    default:
      throw std::invalid_argument("Cannot export an unknown URDF joint type.");
  }
  exportOrigin(joint, _joint.parent_to_joint_origin_transform);
  joint->InsertNewChildElement("parent")->SetAttribute("link", _joint.parent_link_name.c_str());
  joint->InsertNewChildElement("child")->SetAttribute("link", _joint.child_link_name.c_str());
  if (_joint.type != ::urdf::Joint::FIXED && _joint.type != ::urdf::Joint::FLOATING) {
    setVector(joint->InsertNewChildElement("axis"), "xyz", _joint.axis);
  }
  if (_joint.limits) {
    const auto limit = joint->InsertNewChildElement("limit");
    setFiniteAttribute(limit, "lower", _joint.limits->lower);
    setFiniteAttribute(limit, "upper", _joint.limits->upper);
    setFiniteAttribute(limit, "effort", _joint.limits->effort);
    setFiniteAttribute(limit, "velocity", _joint.limits->velocity);
    setFiniteAttribute(limit, "acceleration", _joint.limits->acceleration);
    setFiniteAttribute(limit, "deceleration", _joint.limits->deceleration);
    setFiniteAttribute(limit, "jerk", _joint.limits->jerk);
  }
  if (_joint.dynamics) {
    const auto dynamics = joint->InsertNewChildElement("dynamics");
    setScalar(dynamics, "damping", _joint.dynamics->damping);
    setScalar(dynamics, "friction", _joint.dynamics->friction);
  }
  if (_joint.safety) {
    const auto safety = joint->InsertNewChildElement("safety_controller");
    setScalar(safety, "soft_lower_limit", _joint.safety->soft_lower_limit);
    setScalar(safety, "soft_upper_limit", _joint.safety->soft_upper_limit);
    setScalar(safety, "k_position", _joint.safety->k_position);
    setScalar(safety, "k_velocity", _joint.safety->k_velocity);
  }
  if (_joint.calibration) {
    const auto calibration = joint->InsertNewChildElement("calibration");
    if (_joint.calibration->rising) {
      setScalar(calibration, "rising", *_joint.calibration->rising);
    }
    if (_joint.calibration->falling) {
      setScalar(calibration, "falling", *_joint.calibration->falling);
    }
  }
  if (_joint.mimic) {
    const auto mimic = joint->InsertNewChildElement("mimic");
    mimic->SetAttribute("joint", _joint.mimic->joint_name.c_str());
    setScalar(mimic, "multiplier", _joint.mimic->multiplier);
    setScalar(mimic, "offset", _joint.mimic->offset);
  }
}
}  // namespace

std::unique_ptr<tinyxml2::XMLDocument> exportUrdf(const ::urdf::ModelInterface& _model)
{
  // urdfdom 6 no longer exports XML, so serialize its model with TinyXML2.
  auto doc = std::make_unique<tinyxml2::XMLDocument>();
  const auto robot = doc->NewElement("robot");
  doc->InsertEndChild(robot);
  robot->SetAttribute("name", _model.getName().c_str());
  robot->SetAttribute("version", "1.2");
  for (const auto& [name, material] : _model.materials_) {
    exportMaterial(robot, *material);
  }
  for (const auto& [name, link] : _model.links_) {
    exportLink(robot, *link);
  }
  for (const auto& [name, joint] : _model.joints_) {
    exportJoint(robot, *joint);
  }
  return doc;
}
}  // namespace tobas::urdf
