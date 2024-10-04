#include <iostream>

#include <tobas_path_tools/core.hpp>

#include "../include/tobas_property_tree/property_tree.hpp"

using namespace std;
namespace fs = filesystem;

namespace ptree
{
PropertyTree::PropertyTree()
{
}

bool PropertyTree::initialize(const fs::path& file_path)
{
  if (fs::is_regular_file(file_path))
  {
    // If configuration file exists, try to load it.
    try
    {
      boost::property_tree::ini_parser::read_ini(file_path, pt_);
    }
    catch (...)
    {
      cerr << file_path << " exists, but failed to load it." << endl;
      return false;
    }
  }
  else
  {
    // If configuration file does not exist, create a new one.
    cout << file_path << " does not exist. Creating..." << endl;
    if (!path::createFilePath(file_path, false))
    {
      cerr << "Failed to create " << file_path << "." << endl;
      return false;
    }
  }

  file_path_ = file_path;
  return true;
}

bool PropertyTree::save()
{
  try
  {
    boost::property_tree::ini_parser::write_ini(file_path_, pt_);
  }
  catch (...)
  {
    cerr << "Failed to load " << file_path_ << "." << endl;
    return false;
  }

  return true;
}
}  // namespace ptree
