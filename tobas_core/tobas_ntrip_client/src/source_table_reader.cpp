#include "tobas_ntrip_client/source_table_reader.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>

namespace ntrip
{
SourceTableReader::SourceTableReader(const std::string& data)
{
  std::stringstream data_stream(data);
  std::string moint_point_data;
  while (std::getline(data_stream, moint_point_data)) {
    if (
      moint_point_data.substr(0, 3) ==
      "STR") {  // SOURCETABLE 200 OKなどのserver以外のデータを表す行は保存しない．serverのデータなら最初はSTRとなっているはず．
      std::vector<std::string> moint_point_data_splitted;
      std::stringstream moint_point_data_stream(moint_point_data);
      std::string buf;
      while (std::getline(moint_point_data_stream, buf, ';')) {
        moint_point_data_splitted.push_back(buf);
      }
      auto splitted_size = moint_point_data_splitted.size();
      if (
        splitted_size != kBITRATE + 1 ||
        splitted_size != kMISC + 1) {  // データの個数がbit rateまでのサイズか，miscまでのサイズの場合は推定正しい
        moint_points_.push_back(moint_point_data_splitted);
      }
    }
  }
  std::cout << "Read " << moint_points_.size() << " mount points." << std::endl;
}

std::vector<std::string> SourceTableReader::sortMountPoints(const double& latitude, const double& longitude)
{
  std::vector<std::pair<double, std::string>> distance_and_moint_point_names;

  // 与えられた緯度・経度から各mount pointへの距離を計算して保持する
  for (size_t i = 0; i < moint_points_.size(); i++) {
    double longitude_of_moint_point = 0.0;
    double latitude_of_moint_point = 0.0;
    try {
      longitude_of_moint_point = std::stod(moint_points_[i][kLONGITUDE]);
    }
    catch (const std::exception& e) {
      std::cerr << "Moint point longitude data is strange!" << std::endl;
    }
    try {
      latitude_of_moint_point = std::stod(moint_points_[i][kLATITUDE]);
    }
    catch (const std::exception& e) {
      std::cerr << "Moint point latitude data is strange!" << std::endl;
    }

    // moint_pointの単位球に写像したときの座標を計算
    auto moint_point_x =
      std::cos(latitude_of_moint_point * kDEG_TO_RAD) * std::cos(longitude_of_moint_point * kDEG_TO_RAD);
    auto moint_point_y =
      std::cos(latitude_of_moint_point * kDEG_TO_RAD) * std::sin(longitude_of_moint_point * kDEG_TO_RAD);
    auto moint_point_z = std::sin(latitude_of_moint_point * kDEG_TO_RAD);
    // 指定した緯度・経度の点を単位球へ写像したときの座標を計算
    auto query_point_x = std::cos(latitude * kDEG_TO_RAD) * std::cos(longitude * kDEG_TO_RAD);
    auto query_point_y = std::cos(latitude * kDEG_TO_RAD) * std::sin(longitude * kDEG_TO_RAD);
    auto query_point_z = std::sin(latitude * kDEG_TO_RAD);
    // 距離を計算
    double distance_squared = std::pow((moint_point_x - query_point_x), 2) +
                              std::pow((moint_point_y - query_point_y), 2) +
                              std::pow((moint_point_z - query_point_z), 2);
    distance_and_moint_point_names.emplace_back(distance_squared, moint_points_[i][kMOUNT_POINT]);
  }

  // 距離の小さい順でsort
  std::sort(distance_and_moint_point_names.begin(), distance_and_moint_point_names.end());

  // moint pointの名前だけ取り出して返す
  std::vector<std::string> moint_points_sorted(moint_points_.size());
  for (size_t i = 0; i < moint_points_sorted.size(); i++) {
    moint_points_sorted[i] = distance_and_moint_point_names[i].second;
  }
  return moint_points_sorted;
}

}  // namespace ntrip
