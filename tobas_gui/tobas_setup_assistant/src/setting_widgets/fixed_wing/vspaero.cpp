// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/fixed_wing/vspaero.hpp"

#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
std::expected<VspaeroData, std::string> parseVspaero(const std::string& stab_path)
{
  constexpr char CL[] = "CL";
  constexpr char CD[] = "CD";
  constexpr char CS[] = "CS";
  constexpr char CMl[] = "CMl";
  constexpr char CMm[] = "CMm";
  constexpr char CMn[] = "CMn";

  std::ifstream file(stab_path);

  if (!file.is_open()) {
    return std::unexpected("Failed to open '" + stab_path + "'.");
  }

  std::map<std::string, bool> line_found = {
    { CL, false }, { CD, false }, { CS, false }, { CMl, false }, { CMm, false }, { CMn, false },
  };

  VspaeroData data;
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string name, base, alpha, beta, p, q, r, mach, u;

    if (!(iss >> name >> base >> alpha >> beta >> p >> q >> r >> mach >> u)) {
      continue;
    }

    try {
      if (name == CL) {
        line_found.at(CL) = true;
        data.c_lift_0 = std::stod(base);
        data.c_lift_alpha = std::stod(alpha);
      }
      else if (name == CD) {
        line_found.at(CD) = true;
        data.c_drag_0 = std::stod(base);
        data.c_drag_alpha = std::stod(alpha);
      }
      else if (name == CS) {
        line_found.at(CS) = true;
        data.c_side_beta = std::stod(beta);
      }
      else if (name == CMl) {
        line_found.at(CMl) = true;
        data.c_roll_beta = std::stod(beta);
        data.c_roll_p = std::stod(p);
        data.c_roll_r = std::stod(r);
      }
      else if (name == CMm) {
        line_found.at(CMm) = true;
        data.c_pitch_0 = std::stod(base);
        data.c_pitch_alpha = std::stod(alpha);
        data.c_pitch_abs_beta = std::stod(beta);
        data.c_pitch_alpha_rate = 0.0;
        data.c_pitch_q = std::stod(q);
      }
      else if (name == CMn) {
        line_found.at(CMn) = true;
        data.c_yaw_beta = std::stod(beta);
        data.c_yaw_p = std::stod(p);
        data.c_yaw_r = std::stod(r);
      }
    }
    catch (const std::invalid_argument&) {
      return std::unexpected("Invalid numeric value in '" + name + "' line.");
    }
    catch (const std::out_of_range&) {
      return std::unexpected("Numeric value out of range in '" + name + "' line.");
    }
  }

  if (file.bad()) {
    return std::unexpected("Failed to read '" + stab_path + "'.");
  }

  for (const auto& [line_name, found] : line_found) {
    if (!found) {
      return std::unexpected("'" + line_name + "' line is not found.");
    }
  }

  return data;
}
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
