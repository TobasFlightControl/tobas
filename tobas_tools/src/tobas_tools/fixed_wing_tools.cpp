#include <dh_std_tools/unordered_set.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include "../../include/tobas_tools/fixed_wing_tools.hpp"

using namespace std;
using namespace Eigen;

void getVehicleParameters(VehicleParameters& des)
{
  const string prefix = "/fixed_wing/vehicle";

  des.wing_surface = dh_ros::getParam<double>(prefix + "/wing_surface");
  if (des.wing_surface <= 0.)
  {
    throw dh_ros::RuntimeError("wing_surface must be positive.");
  }

  des.wing_span = dh_ros::getParam<double>(prefix + "/wing_span");
  if (des.wing_span <= 0.)
  {
    throw dh_ros::RuntimeError("wing_span must be positive.");
  }

  des.mean_aerodynamic_chord = dh_ros::getParam<double>(prefix + "/mean_aerodynamic_chord");
  if (des.mean_aerodynamic_chord <= 0.)
  {
    throw dh_ros::RuntimeError("mean_aerodynamic_chord must be positive.");
  }

  auto aerodynamic_center = dh_ros::getParam<vector<double>>(prefix + "/aerodynamic_center");
  if (aerodynamic_center.size() != 3)
  {
    throw dh_ros::RuntimeError("Size mismatch: The size of aerodynamic_center must be 3.");
  }
  des.aerodynamic_center = Map<Vector3d>(aerodynamic_center.data());

  des.alpha_limit.lower = dh_ros::getParam<double>(prefix + "/alpha_limit/lower");
  des.alpha_limit.upper = dh_ros::getParam<double>(prefix + "/alpha_limit/upper");
  if (!des.alpha_limit.isValid())
  {
    throw dh_ros::RuntimeError("Invalid stall angles");
  }
}

void getAerodynamicsCoefficients(AerodynamicsCoefficients& des)
{
  const string prefix = "/fixed_wing/aerodynamic_coefficients";

  des.c_lift_0 = dh_ros::getParam<double>(prefix + "/c_lift_0");
  if (des.c_lift_0 <= 0.)
  {
    throw dh_ros::RuntimeError("c_lift_0 must be positive.");
  }

  des.c_lift_alpha = dh_ros::getParam<double>(prefix + "/c_lift_alpha");
  if (des.c_lift_alpha <= 0.)
  {
    throw dh_ros::RuntimeError("c_lift_alpha must be positive.");
  }

  des.c_drag_0 = dh_ros::getParam<double>(prefix + "/c_drag_0");
  if (des.c_drag_0 <= 0.)
  {
    throw dh_ros::RuntimeError("c_drag_0 must be positive.");
  }

  des.c_drag_alpha = dh_ros::getParam<double>(prefix + "/c_drag_alpha");
  if (des.c_drag_alpha <= 0.)
  {
    throw dh_ros::RuntimeError("c_drag_alpha must be positive.");
  }

  des.c_side_beta = dh_ros::getParam<double>(prefix + "/c_side_beta");
  if (des.c_side_beta >= 0.)
  {
    throw dh_ros::RuntimeError("c_side_beta must be negative.");
  }

  // TODO: 安定微係数の符号チェック
  des.c_roll_beta = dh_ros::getParam<double>(prefix + "/c_roll_beta");
  des.c_roll_p = dh_ros::getParam<double>(prefix + "/c_roll_p");
  des.c_roll_r = dh_ros::getParam<double>(prefix + "/c_roll_r");

  des.c_pitch_0 = dh_ros::getParam<double>(prefix + "/c_pitch_0");
  des.c_pitch_alpha = dh_ros::getParam<double>(prefix + "/c_pitch_alpha");
  des.c_pitch_abs_beta = dh_ros::getParam<double>(prefix + "/c_pitch_abs_beta");
  des.c_pitch_alpha_rate = dh_ros::getParam<double>(prefix + "/c_pitch_alpha_rate");
  des.c_pitch_q = dh_ros::getParam<double>(prefix + "/c_pitch_q");

  des.c_yaw_beta = dh_ros::getParam<double>(prefix + "/c_yaw_beta");
  des.c_yaw_p = dh_ros::getParam<double>(prefix + "/c_yaw_p");
  des.c_yaw_r = dh_ros::getParam<double>(prefix + "/c_yaw_r");
}

void getControlSurfaces(ControlSurfaces& des)
{
  const int num_cs = dh_ros::getParam<int>("/num_control_surfaces");
  des.resize(num_cs);
  unordered_set<uint32_t> indexes;

  for (int i = 0; i < num_cs; ++i)
  {
    const string prefix = "/control_surface_" + to_string(i);

    des[i].index = dh_ros::getParam<int>(prefix + "/index");
    if (des[i].index < 0)
    {
      throw dh_ros::RuntimeError("Please specify non-negative index.");
    }
    else if (dh_std::contains(indexes, des[i].index))
    {
      throw dh_ros::RuntimeError("The index of each control surface must be unique.");
    }

    des[i].angle_limit.lower = dh_ros::getParam<double>(prefix + "/angle_limit/lower");
    des[i].angle_limit.upper = dh_ros::getParam<double>(prefix + "/angle_limit/upper");
    if (!des[i].angle_limit.isValid() || !des[i].angle_limit.inRange(0.))
    {
      throw dh_ros::RuntimeError("Invalid range of control surface angle");
    }

    des[i].c_lift_delta = dh_ros::getParam<double>(prefix + "/c_lift_delta");
    des[i].c_drag_abs_delta = dh_ros::getParam<double>(prefix + "/c_drag_abs_delta");
    des[i].c_side_delta = dh_ros::getParam<double>(prefix + "/c_side_delta");
    des[i].c_roll_delta = dh_ros::getParam<double>(prefix + "/c_roll_delta");
    des[i].c_pitch_delta = dh_ros::getParam<double>(prefix + "/c_pitch_delta");
    des[i].c_yaw_delta = dh_ros::getParam<double>(prefix + "/c_yaw_delta");

    indexes.emplace(des[i].index);
  }
}

void getFixedWingConfig(FixedWingConfig& des)
{
  getVehicleParameters(des.vehicle);
  getAerodynamicsCoefficients(des.aerodynamics);
  getControlSurfaces(des.control_surfaces);
}
