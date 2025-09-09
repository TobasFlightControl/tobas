#include "tobas_udev/core.hpp"

using namespace std;
namespace fs = filesystem;

namespace udv
{
namespace
{
string sanitize(const char* s)
{
  if (!s) {
    return {};
  }

  string out(s);
  for (char& c : out) {
    if (c == '\n' || c == '\t') {
      c = ' ';
    }
  }

  return out;
}
}  // namespace

string getDevNode(udev_device* dev)
{
  const auto devnode = udev_device_get_devnode(dev);
  if (!devnode) {
    return {};
  }
  return string(devnode);
}

string getPropertyValue(udev_device* dev, const char* key)
{
  const auto value = udev_device_get_property_value(dev, key);
  if (!value) {
    return {};
  }
  return string(value);
}

string getSysAttrValue(udev_device* dev, const char* attr)
{
  const auto value = udev_device_get_sysattr_value(dev, attr);
  if (!value) {
    return {};
  }
  return sanitize(value);
}

string getBlockLabel(udev* u, const fs::path& devnode)
{
  const auto sysname = devnode.filename().c_str();
  const auto dev = udev_device_new_from_subsystem_sysname(u, "block", sysname);
  if (!dev) {
    return {};
  }
  return getPropertyValue(dev, "ID_FS_LABEL");
}
}  // namespace udv
