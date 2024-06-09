#include <tobas_std_tools/file.hpp>
#include <tobas_std_tools/unix.hpp>

#include "../include/tobas_std_tools/property_tree.hpp"

using namespace std;

namespace tobas_std
{
PropertyTree::PropertyTree(const string& ini_path, const string& section)
  : ini_path_(tobas_std::expandUser(ini_path)), section_(section)
{
  PRINT_DEBUG("PropertyTree::PropertyTree(" << ini_path << ", " << section << ")");

  if (tobas_std::fileExists(ini_path_))
  {
    load();
  }
  else
  {
    tobas_std::createFilePath(ini_path_);
    PRINT_INFO(ini_path_ << " has been created.");
  }
}

const string& PropertyTree::configPath()
{
  return ini_path_;
}

void PropertyTree::load()
{
  boost::property_tree::ini_parser::read_ini(ini_path_, pt_);
}

void PropertyTree::save()
{
  boost::property_tree::ini_parser::write_ini(ini_path_, pt_);
}

string PropertyTree::keyWithSection(const string& key) const
{
  return section_ + "." + key;
}
}  // namespace tobas_std
