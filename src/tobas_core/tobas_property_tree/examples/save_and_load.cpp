// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <ostream>
#include <string>
#include <vector>

#include <tobas_property_tree/property_tree.hpp>

/** Write vector elements separated by spaces. */
template <typename T>
std::ostream& operator<<(std::ostream& _os, const std::vector<T>& _vec)
{
  const auto size = _vec.size();

  if (size == 0) {
    _os << "[]";
    return _os;
  }

  _os << "[";
  for (std::size_t i = 0; i < size - 1; ++i) {
    _os << _vec[i] << " ";
  }
  _os << _vec[size - 1] << "]";

  return _os;
}

int main()
{
  constexpr char kIntKey[] = "ns.int";
  constexpr char kDoubleKey[] = "ns.double";
  constexpr char kStringKey[] = "ns.string";
  constexpr char kIntListKey[] = "ns.int_list";
  constexpr char kDoubleListKey[] = "ns.double_list";
  constexpr char kStringListKey[] = "ns.string_list";

  tobas::ptree::PropertyTree pt;

  if (const auto result = pt.initialize("/tmp/example.json"); !result) {
    std::cerr << result.error() << '\n';
    return EXIT_FAILURE;
  }

  pt.set(kIntKey, 1);
  pt.set(kDoubleKey, 1.5);
  pt.set(kStringKey, "This is a text.");
  pt.set<int>(kIntListKey, { 1, 2, 3, 4, 5 });
  pt.set<double>(kDoubleListKey, { 1.1, 2.2, 3.3, 4.4, 5.5 });
  pt.set<std::string>(kStringListKey, { "apple", "orange", "banana" });

  if (const auto result = pt.save(); !result) {
    std::cerr << result.error() << '\n';
    return EXIT_FAILURE;
  }

  int int_value;
  double double_value;
  std::string string_value;
  std::vector<int> int_list;
  std::vector<double> double_list;
  std::vector<std::string> string_list;

  if (!pt.get(kIntKey, int_value)) {
    std::cerr << "Failed to get " << kIntKey << ".\n";
    return EXIT_FAILURE;
  }
  std::cout << kIntKey << ": " << int_value << '\n';

  if (!pt.get(kDoubleKey, double_value)) {
    std::cerr << "Failed to get " << kDoubleKey << ".\n";
    return EXIT_FAILURE;
  }
  std::cout << kDoubleKey << ": " << double_value << '\n';

  if (!pt.get(kStringKey, string_value)) {
    std::cerr << "Failed to get " << kStringKey << ".\n";
    return EXIT_FAILURE;
  }
  std::cout << kStringKey << ": " << string_value << '\n';

  if (!pt.get(kIntListKey, int_list)) {
    std::cerr << "Failed to get " << kIntListKey << ".\n";
    return EXIT_FAILURE;
  }
  std::cout << kIntListKey << ": " << int_list << '\n';

  if (!pt.get(kDoubleListKey, double_list)) {
    std::cerr << "Failed to get " << kDoubleListKey << ".\n";
    return EXIT_FAILURE;
  }
  std::cout << kDoubleListKey << ": " << double_list << '\n';

  if (!pt.get(kStringListKey, string_list)) {
    std::cerr << "Failed to get " << kStringListKey << ".\n";
    return EXIT_FAILURE;
  }
  std::cout << kStringListKey << ": " << string_list << '\n';

  return EXIT_SUCCESS;
}
