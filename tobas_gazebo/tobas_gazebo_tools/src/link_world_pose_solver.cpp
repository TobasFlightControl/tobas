#include "../include/tobas_gazebo_tools/link_world_pose_solver.hpp"

using namespace std;
using namespace gz;

namespace gazebo
{
LinkWorldPoseSolver::LinkWorldPoseSolver()
{
}

bool LinkWorldPoseSolver::initialize(const sim::Entity& model, const sim::EntityComponentManager& ecm)
{
  if (!ecm.Component<sim::components::Model>(model)) {
    ignerr << "Model does not exist." << endl;
    return false;
  }

  model_ = model;
  return true;
}

bool LinkWorldPoseSolver::solve(const sim::Entity& link, const sim::EntityComponentManager& ecm)
{
  const auto world = ecm.ParentEntity(model_);
  if (!ecm.Component<sim::components::World>(world)) {
    ignerr << "Model must be a direct child of the world." << endl;
    return false;
  }

  if (ecm.ParentEntity(link) != model_) {
    ignerr << "Link must be a direct child of the model." << endl;
    return false;
  }

  const auto T1 = ecm.Component<sim::components::Pose>(model_);
  if (!T1) {
    ignerr << "Model does not have a pose component." << endl;
    return false;
  }

  const auto T2 = ecm.Component<sim::components::Pose>(link);
  if (!T2) {
    ignerr << "Link does not have a pose component." << endl;
    return false;
  }

  pose_out_ = T1->Data() * T2->Data();
  return true;
}
}  // namespace gazebo
