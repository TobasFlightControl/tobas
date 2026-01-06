#include "./TobasMarkerManager.hpp"

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/math/Rand.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>

#include <tobas_gazebo_common/constants.hpp>

#include "tobas_gazebo_gui_plugins/utils.hpp"

namespace ch = std::chrono;

namespace gazebo
{
TobasMarkerManager::TobasMarkerManager()
{
}

void TobasMarkerManager::LoadConfig(const tinyxml2::XMLElement* _elem)
{
  if (title.empty()) {
    title = "Marker Manager";
  }

  if (_elem) {
    // TODO: Get XML parameters
  }

  gz::gui::App()->findChild<gz::gui::MainWindow*>()->installEventFilter(this);
}

bool TobasMarkerManager::eventFilter(QObject* _obj, QEvent* _event)
{
  if (_event->type() == gz::gui::events::Render::kType) {
    onRender();
  }

  return super::eventFilter(_obj, _event);
}

void TobasMarkerManager::onRender()
{
  if (!scene_) {
    scene_ = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene_) {
      return;
    }

    initialize();
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Process the marker messages
  for (auto marker_it = markers_.begin(); marker_it != markers_.end();) {
    processMarkerMsg(*marker_it);
    markers_.erase(marker_it++);
  }

  // Erase any markers that have a lifetime
  for (auto mit = visuals_.begin(); mit != visuals_.end();) {
    for (auto it = mit->second.cbegin(); it != mit->second.cend(); ++it) {
      if (it->second->GeometryCount() == 0) {
        continue;
      }

      const auto marker_ptr = std::dynamic_pointer_cast<gz::rendering::Marker>(it->second->GeometryByIndex(0));
      if (marker_ptr) {
        if (marker_ptr->Lifetime().count() != 0 && (marker_ptr->Lifetime() <= sim_time_ || sim_time_ < last_sim_time_)) {
          scene_->DestroyVisual(it->second);
          it = mit->second.erase(it);
          break;
        }
      }
    }

    // Erase a namespace if it's empty
    if (mit->second.empty()) {
      mit = visuals_.erase(mit);
    }
    else {
      ++mit;
    }
  }

  last_sim_time_ = sim_time_;
}

void TobasMarkerManager::initialize()
{
  if (!scene_) {
    gzerr << "Scene pointer is invalid" << std::endl;
    return;
  }

  const auto world_name = getWorldName();
  if (!world_name) {
    gzerr << world_name.error() << std::endl;
    return;
  }
  const auto stats_topic = "/world/" + world_name.value() + "/stats";

  node_.Subscribe(kGzMarkerTopic, &self::markerCb, this);
  node_.Subscribe(stats_topic, &self::worldStatsCb, this);

  node_.Advertise(kGzListMarkersSrv, &self::listCb, this);
}

bool TobasMarkerManager::processMarkerMsg(const gz::msgs::Marker& _msg)
{
  // Get the namespace, if it exists. Otherwise, use the global namespace.
  const auto ns = _msg.ns();

  // Get the namespace that the marker belongs to
  auto ns_it = visuals_.find(ns);

  size_t id;
  // If an id is given
  if (_msg.id() != 0) {
    id = _msg.id();
  }
  // Otherwise generate unique id
  else {
    id = gz::math::Rand::IntUniform(0, gz::math::MAX_I32);

    // Make sure it's unique if namespace is given
    if (ns_it != visuals_.end()) {
      while (ns_it->second.contains(id)) {
        id = gz::math::Rand::IntUniform(gz::math::MIN_UI32, gz::math::MAX_UI32);
      }
    }
  }

  // Get visual for this namespace and id
  std::map<uint64_t, gz::rendering::VisualPtr>::iterator visual_it;
  if (ns_it != visuals_.end()) {
    visual_it = ns_it->second.find(id);
  }

  // Add/modify a marker
  if (_msg.action() == gz::msgs::Marker::ADD_MODIFY) {
    // Modify an existing marker, identified by namespace and id
    if (ns_it != visuals_.end() && visual_it != ns_it->second.end()) {
      if (visual_it->second->GeometryCount() > 0) {
        // TODO: Update so that multiple markers can be attached to one visual
        const auto marker_ptr = std::dynamic_pointer_cast<gz::rendering::Marker>(visual_it->second->GeometryByIndex(0));

        visual_it->second->RemoveGeometryByIndex(0);

        // Set the visual values from the Marker Message
        setVisual(_msg, visual_it->second);

        // Set the marker values from the Marker Message
        setMarker(_msg, marker_ptr);

        visual_it->second->AddGeometry(marker_ptr);
      }
    }
    // Otherwise create a new marker
    else {
      // Create the name for the marker
      const auto name = "__GZ_MARKER_VISUAL_" + ns + "_" + std::to_string(id);

      // Create the new marker
      const auto visual_ptr = scene_->CreateVisual(name);

      // Create and load the marker
      const auto marker_ptr = scene_->CreateMarker();

      // Set the visual values from the Marker Message
      setVisual(_msg, visual_ptr);

      // Set the marker values from the Marker Message
      setMarker(_msg, marker_ptr);

      // Add populated marker to the visual
      visual_ptr->AddGeometry(marker_ptr);

      // Add visual to root visual
      if (!visual_ptr->HasParent()) {
        scene_->RootVisual()->AddChild(visual_ptr);
      }

      // Store the visual
      visuals_[ns][id] = visual_ptr;
    }
  }
  // Remove a single marker
  else if (_msg.action() == gz::msgs::Marker::DELETE_MARKER) {
    // Remove the marker if it can be found.
    if (ns_it != visuals_.end() && visual_it != ns_it->second.end()) {
      scene_->DestroyVisual(visual_it->second);
      visuals_[ns].erase(visual_it);

      // Remove namespace if empty
      if (visuals_[ns].empty()) {
        visuals_.erase(ns_it);
      }
    }
    else {
      gzwarn << "Unable to delete marker with id[" << id << "] " << "in namespace[" << ns << "]" << std::endl;
      return false;
    }
  }
  // Remove all markers, or all markers in a namespace
  else if (_msg.action() == gz::msgs::Marker::DELETE_ALL) {
    // If given namespace doesn't exist
    if (!ns.empty() && ns_it == visuals_.end()) {
      gzwarn << "Unable to delete all markers in namespace[" << ns << "], namespace can't be found." << std::endl;
      return false;
    }
    // Remove all markers in the specified namespace
    else if (ns_it != visuals_.end()) {
      for (const auto& it : ns_it->second) {
        scene_->DestroyVisual(it.second);
      }
      ns_it->second.clear();
      visuals_.erase(ns_it);
    }
    // Remove all markers in all namespaces.
    else {
      for (ns_it = visuals_.begin(); ns_it != visuals_.end(); ++ns_it) {
        for (const auto& it : ns_it->second) {
          scene_->DestroyVisual(it.second);
        }
      }
      visuals_.clear();
    }
  }
  else {
    gzerr << "Unknown marker action[" << _msg.action() << "]" << std::endl;
    return false;
  }

  return true;
}

void TobasMarkerManager::setVisual(const gz::msgs::Marker& _msg, const gz::rendering::VisualPtr& _visual_ptr)
{
  // Set Visual Scale
  // The scale for points is used as the size of each point, so skip it here.
  if (_msg.has_scale() && _msg.type() != gz::msgs::Marker::POINTS) {
    _visual_ptr->SetLocalScale(_msg.scale().x(), _msg.scale().y(), _msg.scale().z());
  }

  // Set Visual Pose
  if (_msg.has_pose()) {
    gz::math::Pose3d pose(
      _msg.pose().position().x(),
      _msg.pose().position().y(),
      _msg.pose().position().z(),
      _msg.pose().orientation().w(),
      _msg.pose().orientation().x(),
      _msg.pose().orientation().y(),
      _msg.pose().orientation().z());
    pose.Correct();
    _visual_ptr->SetLocalPose(pose);
  }

  // Set Visual Parent
  if (!_msg.parent().empty()) {
    if (_visual_ptr->HasParent()) {
      _visual_ptr->Parent()->RemoveChild(_visual_ptr);
    }

    const auto parent = scene_->VisualByName(_msg.parent());

    if (parent) {
      parent->AddChild(_visual_ptr);
    }
    else {
      gzerr << "No visual with the name[" << _msg.parent() << "]" << std::endl;
    }
  }

  // TODO: Update Marker Visibility
}

void TobasMarkerManager::setMarker(const gz::msgs::Marker& _msg, const gz::rendering::MarkerPtr& _marker_ptr)
{
  _marker_ptr->SetLayer(_msg.layer());

  // Set Marker Lifetime
  const auto lifetime = ch::seconds(_msg.lifetime().sec()) + ch::nanoseconds(_msg.lifetime().nsec());

  if (lifetime.count() != 0) {
    _marker_ptr->SetLifetime(lifetime + sim_time_);
  }
  else {
    _marker_ptr->SetLifetime(ch::seconds(0));
  }
  // Set Marker Render Type
  const auto marker_type = msgToType(_msg);
  _marker_ptr->SetType(marker_type);

  // Set Marker Material
  if (_msg.has_material()) {
    const auto material_ptr = msgToMaterial(_msg);
    _marker_ptr->SetMaterial(material_ptr, true);

    // Clean up material after clone
    scene_->DestroyMaterial(material_ptr);
  }

  // Assume the presence of points means we clear old ones
  if (_msg.point().size() > 0) {
    _marker_ptr->ClearPoints();
  }

  // Set Marker Points
  for (int i = 0; i < _msg.point().size(); ++i) {
    const gz::math::Vector3d vector(_msg.point(i).x(), _msg.point(i).y(), _msg.point(i).z());
    const auto color = i < _msg.materials().size() ? gz::msgs::Convert(_msg.materials(i).diffuse()) :
                                                     gz::msgs::Convert(_msg.material().diffuse());
    _marker_ptr->AddPoint(vector, color);
  }
  if (_msg.has_scale()) {
    _marker_ptr->SetSize(_msg.scale().x());
  }
}

gz::rendering::MaterialPtr TobasMarkerManager::msgToMaterial(const gz::msgs::Marker& _msg)
{
  const auto material_ptr = scene_->CreateMaterial();

  material_ptr->SetAmbient(
    _msg.material().ambient().r(),
    _msg.material().ambient().g(),
    _msg.material().ambient().b(),
    _msg.material().ambient().a());

  material_ptr->SetDiffuse(
    _msg.material().diffuse().r(),
    _msg.material().diffuse().g(),
    _msg.material().diffuse().b(),
    _msg.material().diffuse().a());

  material_ptr->SetSpecular(
    _msg.material().specular().r(),
    _msg.material().specular().g(),
    _msg.material().specular().b(),
    _msg.material().specular().a());

  material_ptr->SetEmissive(
    _msg.material().emissive().r(),
    _msg.material().emissive().g(),
    _msg.material().emissive().b(),
    _msg.material().emissive().a());

  material_ptr->SetLightingEnabled(_msg.material().lighting());

  return material_ptr;
}

gz::rendering::MarkerType TobasMarkerManager::msgToType(const gz::msgs::Marker& _msg)
{
  const auto new_type = _msg.type();
  if (new_type != cur_type_ && new_type != gz::msgs::Marker::NONE) {
    cur_type_ = _msg.type();
  }

  switch (cur_type_) {
    case gz::msgs::Marker::BOX:
      return gz::rendering::MarkerType::MT_BOX;
    case gz::msgs::Marker::CAPSULE:
      return gz::rendering::MarkerType::MT_CAPSULE;
    case gz::msgs::Marker::CONE:
      return gz::rendering::MarkerType::MT_CONE;
    case gz::msgs::Marker::CYLINDER:
      return gz::rendering::MarkerType::MT_CYLINDER;
    case gz::msgs::Marker::LINE_STRIP:
      return gz::rendering::MarkerType::MT_LINE_STRIP;
    case gz::msgs::Marker::LINE_LIST:
      return gz::rendering::MarkerType::MT_LINE_LIST;
    case gz::msgs::Marker::POINTS:
      return gz::rendering::MarkerType::MT_POINTS;
    case gz::msgs::Marker::SPHERE:
      return gz::rendering::MarkerType::MT_SPHERE;
    case gz::msgs::Marker::TEXT:
      return gz::rendering::MarkerType::MT_TEXT;
    case gz::msgs::Marker::TRIANGLE_FAN:
      return gz::rendering::MarkerType::MT_TRIANGLE_FAN;
    case gz::msgs::Marker::TRIANGLE_LIST:
      return gz::rendering::MarkerType::MT_TRIANGLE_LIST;
    case gz::msgs::Marker::TRIANGLE_STRIP:
      return gz::rendering::MarkerType::MT_TRIANGLE_STRIP;
    default:
      gzerr << "Unable to create marker of type [" << _msg.type() << "]" << std::endl;
      return gz::rendering::MarkerType::MT_NONE;
  }
}

void TobasMarkerManager::markerCb(const gz::msgs::Marker& _msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  markers_.push_back(_msg);
}

void TobasMarkerManager::worldStatsCb(const gz::msgs::WorldStatistics& _msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (_msg.has_sim_time()) {
    sim_time_ = gz::math::secNsecToDuration(_msg.sim_time().sec(), _msg.sim_time().nsec());
  }
  else if (_msg.has_real_time()) {
    sim_time_ = gz::math::secNsecToDuration(_msg.real_time().sec(), _msg.real_time().nsec());
  }
}

bool TobasMarkerManager::listCb(gz::msgs::Marker_V& _req)
{
  std::lock_guard<std::mutex> lock(mutex_);
  _req.clear_marker();

  // Create the list of visuals
  for (const auto& [ns, vm] : visuals_) {
    for (const auto& [id, _] : vm) {
      const auto marker = _req.add_marker();
      marker->set_ns(ns);
      marker->set_id(id);
    }
  }

  return true;
}
}  // namespace gazebo

GZ_ADD_PLUGIN(gazebo::TobasMarkerManager, gz::gui::Plugin)
