// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_components_rt/component_manager.hpp"

#include <class_loader/class_loader.hpp>
#include <rclcpp_components/component_manager.hpp>

namespace tobas
{
namespace
{
// Loader and mutex shared across the entire process.
// Keep loaders per process rather than per class.
std::mutex g_loader_mtx;
std::unordered_map<std::string, std::shared_ptr<class_loader::ClassLoader>> g_loader_cache;
}  // namespace

std::shared_ptr<rclcpp_components::NodeFactory>
ThreadSafeComponentManager::create_component_factory(const ComponentResource& resource)
{
  const auto& library_path = resource.second;
  const auto& class_name = resource.first;
  const auto fq_class_name = "rclcpp_components::NodeFactoryTemplate<" + class_name + ">";

  std::shared_ptr<class_loader::ClassLoader> loader;

  // Guard with a mutex because concurrent `dlopen` calls for the same shared library
  // from multiple `ClassLoader` threads can race.
  {
    const std::lock_guard lock(g_loader_mtx);

    auto it = g_loader_cache.find(library_path);
    if (it == g_loader_cache.end()) {
      RCLCPP_INFO_STREAM(get_logger(), "Load Library: " << library_path);
      try {
        loader = std::make_shared<class_loader::ClassLoader>(library_path);
      }
      catch (const std::exception& e) {
        throw rclcpp_components::ComponentManagerException("Failed to load library: " + std::string(e.what()));
      }
      catch (...) {
        throw rclcpp_components::ComponentManagerException("Failed to load library");
      }
      g_loader_cache.emplace(library_path, loader);
    }
    else {
      loader = it->second;
    }
  }

  const auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
  for (const auto& clazz : classes) {
    RCLCPP_INFO_STREAM(get_logger(), "Found class: " << clazz);
    if (clazz == class_name || clazz == fq_class_name) {
      RCLCPP_INFO_STREAM(get_logger(), "Instantiate class: " << clazz);
      return loader->createInstance<rclcpp_components::NodeFactory>(clazz);
    }
  }

  return {};
}
}  // namespace tobas
