// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Core>

#include "../format.hpp"

namespace YAML
{
template <int Rows, int Cols>  // `Rows` and `Cols` must be `int`, not `Eigen::Index`.
struct convert<Eigen::Matrix<double, Rows, Cols>>
{
  static Node encode(const Eigen::Matrix<double, Rows, Cols>& _rhs)
  {
    static_assert(Rows > 0);
    static_assert(Cols > 0);

    Node node(NodeType::Sequence);

    for (int r = 0; r < Rows; ++r) {
      for (int c = 0; c < Cols; ++c) {
        node.push_back(tobas::yaml::format(_rhs(r, c)));
      }
    }

    return node;
  }

  static bool decode(const Node& _node, Eigen::Matrix<double, Rows, Cols>& _rhs)
  {
    static_assert(Rows > 0);
    static_assert(Cols > 0);

    if (!_node.IsSequence()) {
      return false;
    }
    if (_node.size() != Rows * Cols) {
      return false;
    }

    for (int r = 0; r < Rows; ++r) {
      for (int c = 0; c < Cols; ++c) {
        _rhs(r, c) = _node[r * Cols + c].as<double>();
      }
    }

    return true;
  }
};

template <>
struct convert<Eigen::MatrixXd>
{
  static constexpr char kRowsKey[] = "rows";
  static constexpr char kColsKey[] = "cols";
  static constexpr char kDataKey[] = "data";

  static Node encode(const Eigen::MatrixXd& _rhs)
  {
    const int rows = _rhs.rows();
    const int cols = _rhs.cols();

    Node node(NodeType::Map);

    node[kRowsKey] = rows;
    node[kColsKey] = cols;

    Node data(NodeType::Sequence);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        data.push_back(tobas::yaml::format(_rhs(r, c)));
      }
    }
    node[kDataKey] = data;

    return node;
  }

  static bool decode(const Node& _node, Eigen::MatrixXd& _rhs)
  {
    if (!_node.IsMap()) {
      return false;
    }

    if (!_node[kRowsKey] || !_node[kColsKey] || !_node[kDataKey]) {
      return false;
    }

    const auto rows = _node[kRowsKey].as<int>();
    const auto cols = _node[kColsKey].as<int>();
    const auto data = _node[kDataKey];

    if (data.size() != rows * cols) {
      return false;
    }

    _rhs.conservativeResize(rows, cols);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        _rhs(r, c) = data[r * cols + c].as<double>();
      }
    }

    return true;
  }
};
}  // namespace YAML
