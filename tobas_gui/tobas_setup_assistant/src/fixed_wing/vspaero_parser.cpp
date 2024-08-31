#include <iostream>
#include <fstream>
#include <sstream>
#include <map>

#include "tobas_setup_assistant/setting_tabs/fixed_wing/vspaero_parser.hpp"

using namespace std;

namespace gui
{
namespace setup_assistant
{
bool VSPAEROParser::parse(const string& stab_path)
{
  ifstream file(stab_path);

  if (!file.is_open())
  {
    cerr << "Failed to open \"" << stab_path << "\"." << endl;
    return false;
  }

  map<string, bool> line_found = {
    { CL, false }, { CD, false }, { CS, false }, { CMl, false }, { CMm, false }, { CMn, false },
  };

  string line;
  while (getline(file, line))
  {
    istringstream iss(line);
    string name, base, alpha, beta, p, q, r, mach, u;

    if (!(iss >> name >> base >> alpha >> beta >> p >> q >> r >> mach >> u))
      continue;

    if (name == CL)
    {
      line_found.at(CL) = true;
      c_lift_0_ = stod(base);
      c_lift_alpha_ = stod(alpha);
    }
    else if (name == CD)
    {
      line_found.at(CD) = true;
      c_drag_0_ = stod(base);
      c_drag_alpha_ = stod(alpha);
    }
    else if (name == CS)
    {
      line_found.at(CS) = true;
      c_side_beta_ = stod(beta);
    }
    else if (name == CMl)
    {
      line_found.at(CMl) = true;
      c_roll_beta_ = stod(beta);
      c_roll_p_ = stod(p);
      c_roll_r_ = stod(r);
    }
    else if (name == CMm)
    {
      line_found.at(CMm) = true;
      c_pitch_0_ = stod(base);
      c_pitch_alpha_ = stod(alpha);
      c_pitch_abs_beta_ = stod(beta);
      c_pitch_alpha_rate_ = 0.0;
      c_pitch_q_ = stod(q);
    }
    else if (name == CMn)
    {
      line_found.at(CMn) = true;
      c_yaw_beta_ = stod(beta);
      c_yaw_p_ = stod(p);
      c_yaw_r_ = stod(r);
    }
  }

  for (const auto& [line, found] : line_found)
  {
    if (!found)
    {
      cerr << "\"" << line << "\" line is not found." << endl;
      return false;
    }
  }

  return true;
}

double VSPAEROParser::c_lift_0() const
{
  return c_lift_0_;
}

double VSPAEROParser::c_lift_alpha() const
{
  return c_lift_alpha_;
}

double VSPAEROParser::c_drag_0() const
{
  return c_drag_0_;
}

double VSPAEROParser::c_drag_alpha() const
{
  return c_drag_alpha_;
}

double VSPAEROParser::c_side_beta() const
{
  return c_side_beta_;
}

double VSPAEROParser::c_roll_beta() const
{
  return c_roll_beta_;
}

double VSPAEROParser::c_roll_p() const
{
  return c_roll_p_;
}

double VSPAEROParser::c_roll_r() const
{
  return c_roll_r_;
}

double VSPAEROParser::c_pitch_0() const
{
  return c_pitch_0_;
}

double VSPAEROParser::c_pitch_alpha() const
{
  return c_pitch_alpha_;
}

double VSPAEROParser::c_pitch_abs_beta() const
{
  return c_pitch_abs_beta_;
}

double VSPAEROParser::c_pitch_alpha_rate() const
{
  return c_pitch_alpha_rate_;
}

double VSPAEROParser::c_pitch_q() const
{
  return c_pitch_q_;
}

double VSPAEROParser::c_yaw_beta() const
{
  return c_yaw_beta_;
}

double VSPAEROParser::c_yaw_p() const
{
  return c_yaw_p_;
}

double VSPAEROParser::c_yaw_r() const
{
  return c_yaw_r_;
}
}  // namespace setup_assistant
}  // namespace gui
