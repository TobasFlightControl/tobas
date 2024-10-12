#include <fstream>
#include <iostream>

#include "../include/tobas_yaml_tools/core.hpp"

using namespace std;

namespace yaml
{
string dump(const YAML::Node& node)
{
  YAML::Emitter emitter;
  emitter << node;

  stringstream res;
  res << emitter.c_str() << endl;

  return res.str();
}

bool load(const filesystem::path& path, YAML::Node& node)
{
  if (!filesystem::exists(path))
  {
    cerr << path << " does not exist." << endl;
    return false;
  }

  try
  {
    node = YAML::LoadFile(path);
  }
  catch (const exception& e)
  {
    cerr << "Failed to load yaml: " << e.what() << endl;
    return false;
  }

  return true;
}

bool save(const filesystem::path& path, const YAML::Node& node)
{
  ofstream fout(path);
  if (!fout.is_open())
  {
    cerr << "Failed to open \"" << path << "\" for writing." << endl;
    return false;
  }

  fout << dump(node);
  fout.close();

  return true;
}
}  // namespace yaml
