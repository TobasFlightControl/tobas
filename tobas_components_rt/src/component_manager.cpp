#include "tobas_components_rt/component_manager.hpp"

#include <class_loader/class_loader.hpp>
#include <rclcpp_components/component_manager.hpp>

using namespace std;

namespace
{
// プロセス全体で共有するローダと排他ロック
// ローダをクラス単位ではなくプロセス単位で保持
mutex g_loader_mtx;
unordered_map<string, shared_ptr<class_loader::ClassLoader>> g_loader_cache;
}  // namespace

namespace ros2
{
shared_ptr<rclcpp_components::NodeFactory>
ThreadSafeComponentManager::create_component_factory(const ComponentResource& resource)
{
  const auto& library_path = resource.second;
  const auto& class_name = resource.first;
  const auto fq_class_name = "rclcpp_components::NodeFactoryTemplate<" + class_name + ">";

  shared_ptr<class_loader::ClassLoader> loader;

  // ClassLoaderで1つの共有ライブラリを複数のスレッドが同時にdlopenすると競合するため，排他ロックする必要がある．
  {
    lock_guard<mutex> lock(g_loader_mtx);

    auto it = g_loader_cache.find(library_path);
    if (it == g_loader_cache.end()) {
      RCLCPP_INFO_STREAM(get_logger(), "Load Library: " << library_path);
      try {
        loader = std::make_shared<class_loader::ClassLoader>(library_path);
      }
      catch (const exception& ex) {
        throw rclcpp_components::ComponentManagerException("Failed to load library: " + string(ex.what()));
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
}  // namespace ros2
