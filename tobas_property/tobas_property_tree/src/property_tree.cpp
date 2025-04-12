#include <boost/property_tree/json_parser.hpp>

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
      boost::property_tree::json_parser::read_json(file_path, root_node_);
    }
    catch (const exception& e)
    {
      cerr << file_path << " exists, but failed to load it: " << e.what() << endl;
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
    boost::property_tree::json_parser::write_json(file_path_, root_node_);
  }
  catch (const exception& e)
  {
    cerr << "Failed to save " << file_path_ << ": " << e.what() << endl;
    return false;
  }

  return true;
}
}  // namespace ptree
