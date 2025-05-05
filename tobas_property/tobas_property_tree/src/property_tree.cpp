#include "tobas_property_tree/property_tree.hpp"

#include <boost/property_tree/json_parser.hpp>

#include <tobas_path_tools/core.hpp>

using namespace std;
namespace fs = filesystem;

namespace ptree
{
PropertyTree::PropertyTree()
{
}

bool PropertyTree::initialize(const fs::path& file_path)
{
  if (fs::is_regular_file(file_path)) {
    // ファイルが存在する場合は読み込む
    try {
      boost::property_tree::json_parser::read_json(file_path, root_node_);
      cout << file_path << " is loaded successfully." << endl;
    }
    catch (const exception& e) {
      // 読み込みに失敗したら元のファイルを削除
      cerr << "Failed to load " << file_path << ": " << e.what() << endl;
      cerr << "Removing " << file_path << "." << endl;
      if (!fs::remove(file_path)) {
        cerr << "Failed to remove " << file_path << "." << endl;
        return false;
      }
    }
  }
  else {
    cout << file_path << " does not exist." << endl;
  }

  file_path_ = file_path;
  parent_dir_ = file_path.parent_path();

  return true;
}

bool PropertyTree::save()
{
  // 親ディレクトリが存在しない場合は作成する
  if (!fs::is_directory(parent_dir_)) {
    if (!fs::create_directories(parent_dir_)) {
      cerr << "Failed to create " << parent_dir_ << "." << endl;
      return false;
    }
  }

  try {
    boost::property_tree::json_parser::write_json(file_path_, root_node_);
  }
  catch (const exception& e) {
    cerr << "Failed to save " << file_path_ << ": " << e.what() << endl;
    return false;
  }

  return true;
}
}  // namespace ptree
