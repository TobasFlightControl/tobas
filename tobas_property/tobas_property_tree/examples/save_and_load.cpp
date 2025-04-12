#include <iostream>

#include <tobas_property_tree/property_tree.hpp>

using namespace std;

template <typename T>
ostream& operator<<(ostream& os, const vector<T>& vec)
{
  const auto size = vec.size();

  if (size == 0)
  {
    os << "[]";
    return os;
  }

  os << "[";
  for (size_t i = 0; i < size - 1; ++i)
    os << vec[i] << " ";
  os << vec[size - 1] << "]";

  return os;
}

int main()
{
  static constexpr char kIntKey[] = "int";
  static constexpr char kDoubleKey[] = "double";
  static constexpr char kStringKey[] = "string";
  static constexpr char kIntListKey[] = "int_list";
  static constexpr char kDoubleListKey[] = "double_list";
  static constexpr char kStringListKey[] = "string_list";

  ptree::PropertyTree pt;

  if (!pt.initialize("/tmp/example.json"))
    return EXIT_FAILURE;

  pt.set(kIntKey, 1);
  pt.set(kDoubleKey, 1.5);
  pt.set(kStringKey, "This is a text.");
  pt.set<int>(kIntListKey, { 1, 2, 3, 4, 5 });
  pt.set<double>(kDoubleListKey, { 1.1, 2.2, 3.3, 4.4, 5.5 });
  pt.set<string>(kStringListKey, { "apple", "orange", "banana" });

  if (!pt.save())
    return EXIT_FAILURE;

  int int_value;
  double double_value;
  string string_value;
  vector<int> int_list;
  vector<double> dobule_list;
  vector<string> string_list;

  if (!pt.get(kIntKey, int_value))
    return EXIT_FAILURE;
  cout << kIntKey << ": " << int_value << endl;

  if (!pt.get(kDoubleKey, double_value))
    return EXIT_FAILURE;
  cout << kDoubleKey << ": " << double_value << endl;

  if (!pt.get(kStringKey, string_value))
    return EXIT_FAILURE;
  cout << kStringKey << ": " << string_value << endl;

  if (!pt.get(kIntListKey, int_list))
    return EXIT_FAILURE;
  cout << kIntListKey << ": " << int_list << endl;

  if (!pt.get(kDoubleListKey, dobule_list))
    return EXIT_FAILURE;
  cout << kDoubleListKey << ": " << dobule_list << endl;

  if (!pt.get(kStringListKey, string_list))
    return EXIT_FAILURE;
  cout << kStringListKey << ": " << string_list << endl;

  return EXIT_SUCCESS;
}
