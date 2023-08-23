#include <gazebo/common/Assert.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/ode/ODETypes.hh>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/ode/ODEPhysics.hh>
#include <gazebo/physics/ode/ODERayShape.hh>
#include <gazebo/physics/ode/ODEMultiRayShape.hh>

#include "../include/tobas_gazebo_plugins/ode_multiray_shape.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"

using namespace std;
using namespace gazebo;
using namespace physics;
using namespace ignition::math;

OdeMultiRayShape::OdeMultiRayShape(CollisionPtr parent) : super(parent)
{
  SetName("ODE Multiray Shape");

  // Create a space to contain the ray space
  super_space_id_ = dSimpleSpaceCreate(0);
  ray_space_id_ = dSimpleSpaceCreate(super_space_id_);

  // Set collision bits
  dGeomSetCategoryBits((dGeomID)ray_space_id_, GZ_SENSOR_COLLIDE);
  dGeomSetCollideBits((dGeomID)ray_space_id_, ~GZ_SENSOR_COLLIDE);

  // These three lines may be unessecary
  ODELinkPtr link = boost::static_pointer_cast<ODELink>(collisionParent->GetLink());
  link->SetSpaceId(ray_space_id_);
  boost::static_pointer_cast<ODECollision>(collisionParent)->SetSpaceId(ray_space_id_);
}

OdeMultiRayShape::~OdeMultiRayShape()
{
  dSpaceSetCleanup(ray_space_id_, 0);
  dSpaceDestroy(ray_space_id_);

  dSpaceSetCleanup(super_space_id_, 0);
  dSpaceDestroy(super_space_id_);
}

void OdeMultiRayShape::Init()
{
  getSdfParams();

  y_diff_ = hor_max_angle_ - hor_min_angle_;
  p_diff_ = ver_max_angle_ - ver_min_angle_;
}

void OdeMultiRayShape::UpdateRays()
{
  ODEPhysicsPtr ode = boost::dynamic_pointer_cast<ODEPhysics>(GetWorld()->Physics());

  if (ode == NULL)
    gzthrow("Invalid physics engine. Must use ODE.");

  // Do we need to lock the physics engine here? YES!
  // especially when spawning models with sensors
  {
    boost::recursive_mutex::scoped_lock lock(*ode->GetPhysicsUpdateMutex());

    // Do collision detection
    dSpaceCollide2((dGeomID)(super_space_id_), (dGeomID)(ode->GetSpaceId()), this, &updateCallback);
  }
}

void OdeMultiRayShape::AddRay(const Vector3d& start, const Vector3d& end)
{
  super::AddRay(start, end);

  ODECollisionPtr odeCollision(new ODECollision(collisionParent->GetLink()));
  odeCollision->SetName("ode_ray_collision");
  odeCollision->SetSpaceId(ray_space_id_);

  ODERayShapePtr ray(new ODERayShape(odeCollision));
  odeCollision->SetShape(ray);

  ray->SetPoints(start, end);
  rays.push_back(ray);
}

vector<RayShapePtr>& OdeMultiRayShape::rayShapes()
{
  return rays;
}

void OdeMultiRayShape::getSdfParams()
{
  rayElem = sdf->GetElement("ray");
  scanElem = rayElem->GetElement("scan");
  horzElem = scanElem->GetElement("horizontal");
  rangeElem = rayElem->GetElement("range");

  getSdfParam(rangeElem, "min", min_range_);
  getSdfParam(rangeElem, "max", max_range_);

  getSdfParam(horzElem, "min_angle", hor_min_angle_);
  getSdfParam(horzElem, "max_angle", hor_max_angle_);
  getSdfParam(horzElem, "samples", hor_samples_, kDefaultHorizontalSamples, POSITIVE);

  if (scanElem->HasElement("vertical"))
  {
    vertElem = scanElem->GetElement("vertical");
    getSdfParam(vertElem, "min_angle", ver_min_angle_);
    getSdfParam(vertElem, "max_angle", ver_max_angle_);
    getSdfParam(vertElem, "samples", ver_samples_, kDefaultVerticalSamples, POSITIVE);
  }
}

void OdeMultiRayShape::updateCallback(void* data, dGeomID o1, dGeomID o2)
{
  dContactGeom contact;
  OdeMultiRayShape* self = NULL;

  self = static_cast<OdeMultiRayShape*>(data);

  // Check space
  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
  {
    if (dGeomGetSpace(o1) == self->super_space_id_ || dGeomGetSpace(o2) == self->super_space_id_)
      dSpaceCollide2(o1, o2, self, &updateCallback);

    if (dGeomGetSpace(o1) == self->ray_space_id_ || dGeomGetSpace(o2) == self->ray_space_id_)
      dSpaceCollide2(o1, o2, self, &updateCallback);
  }
  else
  {
    ODECollision* collision1 = NULL;
    ODECollision* collision2 = NULL;

    // Get pointers to the underlying collisions
    if (dGeomGetClass(o1) == dGeomTransformClass)
      collision1 = static_cast<ODECollision*>(dGeomGetData(dGeomTransformGetGeom(o1)));
    else
      collision1 = static_cast<ODECollision*>(dGeomGetData(o1));

    if (dGeomGetClass(o2) == dGeomTransformClass)
      collision2 = static_cast<ODECollision*>(dGeomGetData(dGeomTransformGetGeom(o2)));
    else
      collision2 = static_cast<ODECollision*>(dGeomGetData(o2));

    GZ_ASSERT(collision1, "collision1 is null");
    GZ_ASSERT(collision2, "collision2 is null");

    ODECollision* rayCollision = NULL;
    ODECollision* hitCollision = NULL;

    // Figure out which one is a ray; note that this assumes
    // that the ODE dRayClass is used *soley* by the RayCollision.
    if (dGeomGetClass(o1) == dRayClass)
    {
      rayCollision = static_cast<ODECollision*>(collision1);
      hitCollision = static_cast<ODECollision*>(collision2);
      dGeomRaySetParams(o1, 0, 0);
      dGeomRaySetClosestHit(o1, 1);
    }
    else if (dGeomGetClass(o2) == dRayClass)
    {
      GZ_ASSERT(rayCollision == NULL, "rayCollision is not null");
      rayCollision = static_cast<ODECollision*>(collision2);
      hitCollision = static_cast<ODECollision*>(collision1);
      dGeomRaySetParams(o2, 0, 0);
      dGeomRaySetClosestHit(o2, 1);
    }

    // Check for ray/collision intersections
    if (rayCollision && hitCollision)
    {
      const int n = dCollide(o1, o2, 1, &contact, sizeof(contact));

      if (n > 0)
      {
        RayShapePtr shape = boost::static_pointer_cast<RayShape>(rayCollision->GetShape());
        if (contact.depth < shape->GetLength())
        {
          shape->SetLength(contact.depth);
          shape->SetRetro(hitCollision->GetLaserRetro());
        }
      }
    }
  }
}
