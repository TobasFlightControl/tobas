#include "../include/tobas_std_tools/property_tree.hpp"

using namespace std;

namespace tobas_std
{
PropertyTree::PropertyTree(const string& ini_path, const string& section) : ini_path_(ini_path), section_(section)
{
  PRINT_DEBUG("PropertyTree::PropertyTree(" << ini_path_ << ", " << section_ << ")");
}

const string& PropertyTree::configPath() const
{
  return ini_path_;
}

bool PropertyTree::load()
{
  PRINT_DEBUG("PropertyTree::load");

  try
  {
    boost::property_tree::ini_parser::read_ini(ini_path_, pt_);
  }
  catch (...)
  {
    return false;
  }

  return true;
}

bool PropertyTree::save()
{
  PRINT_DEBUG("PropertyTree::save");

  try
  {
    boost::property_tree::ini_parser::write_ini(ini_path_, pt_);
  }
  catch (...)
  {
    return false;
  }

  return true;
}

string PropertyTree::keyWithSection(const string& key) const
{
  return section_ + "." + key;
}
}  // namespace tobas_std
