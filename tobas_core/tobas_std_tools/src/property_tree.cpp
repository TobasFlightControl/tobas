#include <tobas_std_tools/fstream.hpp>

#include "../include/tobas_std_tools/property_tree.hpp"

namespace tobas_std
{
PropertyTree::PropertyTree(const std::string& ini_path) : ini_path_(ini_path)
{
  if (tobas_std::fileExists(ini_path))
    boost::property_tree::ini_parser::read_ini(ini_path, pt_);
}

void PropertyTree::save()
{
  boost::property_tree::ini_parser::write_ini(ini_path_, pt_);
}
}  // namespace tobas_std
