#include <tobas_std_tools/file.hpp>

#include "../include/tobas_std_tools/property_tree.hpp"

using namespace std;

namespace tobas_std
{
PropertyTree::PropertyTree(const std::string& ini_path) : ini_path_(ini_path)
{
  if (tobas_std::fileExists(ini_path))
    boost::property_tree::ini_parser::read_ini(ini_path, pt_);
}

void PropertyTree::save()
{
  if (!tobas_std::fileExists(ini_path_))
    tobas_std::createFilePath(ini_path_);

  boost::property_tree::ini_parser::write_ini(ini_path_, pt_);
}
}  // namespace tobas_std
