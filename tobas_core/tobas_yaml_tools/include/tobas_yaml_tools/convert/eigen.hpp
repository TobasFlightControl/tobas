#pragma once

#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Core>

namespace YAML
{
// Rows, Colsの型はEigen::Indexではなくintである必要がある
template <typename Scalar, int Rows, int Cols>
struct convert<Eigen::Matrix<Scalar, Rows, Cols>>
{
  static Node encode(const Eigen::Matrix<Scalar, Rows, Cols>& rhs)
  {
    Node node(NodeType::Sequence);

    for (int r = 0; r < Rows; ++r)
      for (int c = 0; c < Cols; ++c)
        node.push_back(rhs(r, c));

    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix<Scalar, Rows, Cols>& rhs)
  {
    if (!node.IsSequence())
      return false;
    if (node.size() != Rows * Cols)
      return false;

    for (int r = 0; r < Rows; ++r)
      for (int c = 0; c < Cols; ++c)
        rhs(r, c) = node[r * Cols + c].as<Scalar>();

    return true;
  }
};
}  // namespace YAML
