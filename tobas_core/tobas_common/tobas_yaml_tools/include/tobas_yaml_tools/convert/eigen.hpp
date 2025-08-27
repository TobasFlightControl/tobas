#pragma once

#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Core>

#include "./util.hpp"

namespace YAML
{
template <int Rows, int Cols>  // Rows, Colsの型はEigen::Indexではなくintである必要がある
struct convert<Eigen::Matrix<double, Rows, Cols>>
{
  static Node encode(const Eigen::Matrix<double, Rows, Cols>& rhs)
  {
    static_assert(Rows > 0);
    static_assert(Cols > 0);

    Node node(NodeType::Sequence);

    for (int r = 0; r < Rows; ++r) {
      for (int c = 0; c < Cols; ++c) {
        node.push_back(util::format(rhs(r, c)));
      }
    }

    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix<double, Rows, Cols>& rhs)
  {
    static_assert(Rows > 0);
    static_assert(Cols > 0);

    if (!node.IsSequence()) {
      return false;
    }
    if (node.size() != Rows * Cols) {
      return false;
    }

    for (int r = 0; r < Rows; ++r) {
      for (int c = 0; c < Cols; ++c) {
        rhs(r, c) = node[r * Cols + c].as<double>();
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

  static Node encode(const Eigen::MatrixXd& rhs)
  {
    const int rows = rhs.rows();
    const int cols = rhs.cols();

    Node node(NodeType::Map);

    node[kRowsKey] = rows;
    node[kColsKey] = cols;

    Node data(NodeType::Sequence);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        data.push_back(util::format(rhs(r, c)));
      }
    }
    node[kDataKey] = data;

    return node;
  }

  static bool decode(const Node& node, Eigen::MatrixXd& rhs)
  {
    if (!node.IsMap()) {
      return false;
    }

    if (!node[kRowsKey] || !node[kColsKey] || !node[kDataKey]) {
      return false;
    }

    const auto rows = node[kRowsKey].as<int>();
    const auto cols = node[kColsKey].as<int>();
    const auto data = node[kDataKey];

    if (data.size() != rows * cols) {
      return false;
    }

    rhs.conservativeResize(rows, cols);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        rhs(r, c) = data[r * cols + c].as<double>();
      }
    }

    return true;
  }
};
}  // namespace YAML
