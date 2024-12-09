#pragma once

#include <unordered_set>
#include <rclcpp/type_adapter.hpp>

#include <tobas_std_tools/debug.hpp>
#include <tobas_kdl/tree.hpp>
#include <tobas_kdl_msgs/msg/tree.hpp>

#include "./segment.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::Tree, tobas_kdl_msgs::msg::Tree>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::Tree;
  using ros_message_type = tobas_kdl_msgs::msg::Tree;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.segments.clear();

    for (const auto& [_, elem] : src.getSegments())
    {
      dst.segments.emplace_back();
      tobas_kdl_msgs::SegmentAdapter::convert_to_ros_message(elem.segment, dst.segments.back().segment);
      dst.segments.back().q_nr = elem.q_nr;

      // ルートリンクでなければ親の名前を追加
      if (elem.segment.name() != src.getRootName())
        dst.segments.back().parent_name = elem.parent->first;
    }

    dst.root_name = src.getRootName();
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    kdl::Tree tree(src.root_name);

    std::unordered_set<std::string> added_segs;  // ツリーに追加されたリンク名
    added_segs.insert(src.root_name);  // ツリーを作成した時点でルートリンクは含まれている

    kdl::Segment seg;
    size_t q_nr = 0;  // 現在の可動関節の番号

    for (size_t _ = 0; _ < src.segments.size(); ++_)
    {
      for (const auto& elem : src.segments)
      {
        // リンクが既に追加されていればスキップ
        if (added_segs.contains(elem.segment.name))
          continue;

        // q_nrの整合性を保つため，可動関節は番号の若い方から順にツリーに追加する．
        // 固定関節をもつリンクの番号は0だから，常に追加候補になる．
        if (elem.q_nr > q_nr)
          continue;

        // 親リンクが追加されていればまだ追加できない
        if (!added_segs.contains(elem.parent_name))
          continue;

        // 現在のリンクをツリーに追加
        tobas_kdl_msgs::SegmentAdapter::convert_to_custom(elem.segment, seg);
        if (!tree.addSegment(seg, elem.parent_name))
          throw std::runtime_error("Failed to add segment \"" + elem.segment.name + "\".");
        added_segs.insert(elem.segment.name);

        // 可動関節の場合は次の番号の関節をもつリンクを探索
        if (elem.segment.joint.type != kdl::Joint::FIXED)
          ++q_nr;
      }

      // 全てのリンクが追加されたらツリーをコピーして終了
      if (added_segs.size() == src.segments.size())
      {
        dst = tree;
        return;
      }
    }

    // 1回のループで最低でも1つリンクが追加されるため，リンク数のループを終えても終了条件を満たさない場合は何かがおかしい．
    throw std::runtime_error("Failed to convert tobas_kdl_msgs/Tree to kdl::Tree.");
  }
};

namespace tobas_kdl_msgs
{
using TreeAdapter = rclcpp::TypeAdapter<kdl::Tree, tobas_kdl_msgs::msg::Tree>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::Tree, tobas_kdl_msgs::msg::Tree);
