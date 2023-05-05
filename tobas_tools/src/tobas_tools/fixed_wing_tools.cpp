#include <dh_std_tools/unordered_set.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include "../../include/tobas_tools/fixed_wing_tools.hpp"

using namespace std;
using namespace Eigen;

void getVehicleParameters(VehicleParameters& des)
{
  const string prefix = "/fixed_wing/vehicle";

  dh_ros::getParam(prefix + "/wing_surface", des.wing_surface);
  if (des.wing_surface <= 0.)
  {
    throw dh_ros::RuntimeError("wing_surface must be positive.");
  }

  dh_ros::getParam(prefix + "/wing_span", des.wing_span);
  if (des.wing_span <= 0.)
  {
    throw dh_ros::RuntimeError("wing_span must be positive.");
  }

  dh_ros::getParam(prefix + "/mean_aerodynamic_chord", des.mean_aerodynamic_chord);
  if (des.mean_aerodynamic_chord <= 0.)
  {
    throw dh_ros::RuntimeError("mean_aerodynamic_chord must be positive.");
  }

  vector<double> aerodynamic_center;
  dh_ros::getParam<vector<double>>(prefix + "/aerodynamic_center", aerodynamic_center);
  if (aerodynamic_center.size() != 3)
  {
    throw dh_ros::RuntimeError("Size mismatch: The size of aerodynamic_center must be 3.");
  }
  des.aerodynamic_center = Map<Vector3d>(aerodynamic_center.data());

  dh_ros::getParam(prefix + "/alpha_limit/lower", des.alpha_limit.lower);
  dh_ros::getParam(prefix + "/alpha_limit/upper", des.alpha_limit.upper);
  if (!des.alpha_limit.isValid())
  {
    throw dh_ros::RuntimeError("Invalid stall angles");
  }
}

void getAerodynamicsCoefficients(AerodynamicsCoefficients& des)
{
  const string prefix = "/fixed_wing/aerodynamic_coefficients";

  dh_ros::getParam(prefix + "/c_lift_0", des.c_lift_0);
  if (des.c_lift_0 <= 0.)
  {
    throw dh_ros::RuntimeError("c_lift_0 must be positive.");
  }

  dh_ros::getParam(prefix + "/c_lift_alpha", des.c_lift_alpha);
  if (des.c_lift_alpha <= 0.)
  {
    throw dh_ros::RuntimeError("c_lift_alpha must be positive.");
  }

  dh_ros::getParam(prefix + "/c_drag_0", des.c_drag_0);
  if (des.c_drag_0 <= 0.)
  {
    throw dh_ros::RuntimeError("c_drag_0 must be positive.");
  }

  dh_ros::getParam(prefix + "/c_drag_alpha", des.c_drag_alpha);
  if (des.c_drag_alpha <= 0.)
  {
    throw dh_ros::RuntimeError("c_drag_alpha must be positive.");
  }

  dh_ros::getParam(prefix + "/c_side_beta", des.c_side_beta);
  if (des.c_side_beta >= 0.)
  {
    throw dh_ros::RuntimeError("c_side_beta must be negative.");
  }

  // TODO: 安定微係数の符号チェック
  dh_ros::getParam(prefix + "/c_roll_beta", des.c_roll_beta);
  dh_ros::getParam(prefix + "/c_roll_p", des.c_roll_p);
  dh_ros::getParam(prefix + "/c_roll_r", des.c_roll_r);

  dh_ros::getParam(prefix + "/c_pitch_0", des.c_pitch_0);
  dh_ros::getParam(prefix + "/c_pitch_alpha", des.c_pitch_alpha);
  dh_ros::getParam(prefix + "/c_pitch_abs_beta", des.c_pitch_abs_beta);
  dh_ros::getParam(prefix + "/c_pitch_alpha_rate", des.c_pitch_alpha_rate);
  dh_ros::getParam(prefix + "/c_pitch_q", des.c_pitch_q);

  dh_ros::getParam(prefix + "/c_yaw_beta", des.c_yaw_beta);
  dh_ros::getParam(prefix + "/c_yaw_p", des.c_yaw_p);
  dh_ros::getParam(prefix + "/c_yaw_r", des.c_yaw_r);
}

void getControlSurfaces(ControlSurfaces& des)
{
  int num_cs;
  dh_ros::getParam("/num_control_surfaces", num_cs);
  des.resize(num_cs);

  unordered_set<int> indexes;

  for (int i = 0; i < num_cs; ++i)
  {
    const string prefix = "/control_surface_" + to_string(i);

    dh_ros::getParam(prefix + "/index", des[i].index);
    if (des[i].index < 0)
    {
      throw dh_ros::RuntimeError("Please specify non-negative index.");
    }
    else if (dh_std::contains(indexes, des[i].index))
    {
      throw dh_ros::RuntimeError("The index of each control surface must be unique.");
    }

    dh_ros::getParam(prefix + "/angle_limit/lower", des[i].angle_limit.lower);
    dh_ros::getParam(prefix + "/angle_limit/upper", des[i].angle_limit.upper);
    if (!des[i].angle_limit.isValid() || !des[i].angle_limit.inRange(0.))
    {
      throw dh_ros::RuntimeError("Invalid range of control surface angle");
    }

    dh_ros::getParam(prefix + "/c_lift_delta", des[i].c_lift_delta);
    dh_ros::getParam(prefix + "/c_drag_abs_delta", des[i].c_drag_abs_delta);
    dh_ros::getParam(prefix + "/c_side_delta", des[i].c_side_delta);
    dh_ros::getParam(prefix + "/c_roll_delta", des[i].c_roll_delta);
    dh_ros::getParam(prefix + "/c_pitch_delta", des[i].c_pitch_delta);
    dh_ros::getParam(prefix + "/c_yaw_delta", des[i].c_yaw_delta);

    indexes.emplace(des[i].index);
  }
}

void getFixedWingConfig(FixedWingConfig& des)
{
  getVehicleParameters(des.vehicle);
  getAerodynamicsCoefficients(des.aerodynamics);
  getControlSurfaces(des.control_surfaces);
}
