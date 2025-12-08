#include "../include/FoilDynamics.hh"
#include <cmath>

namespace foil_dynamics
{

/////////////////////////////////////////////////
FoilDynamics::FoilDynamics()
: rho_(1000.1),
  cp_(0, 0, 0),
  forward_(1, 0, 0),
  upward_(0, 0, 1),
  area_(1.0),
  clmax_(1.5),
  cdmax_(1.0),
  wind_(0, 0, 0)
{
  RCLCPP_INFO(rclcpp::get_logger("FoilDynamics"), "FoilDynamics plugin constructed");
}

/////////////////////////////////////////////////
FoilDynamics::~FoilDynamics() = default;

/////////////////////////////////////////////////
void FoilDynamics::Configure(const gz::sim::Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             gz::sim::EntityComponentManager &_ecm,
                             gz::sim::EventManager &)
{
  model_ = gz::sim::Model(_entity);
  if (!model_.Valid(_ecm))
  {
    RCLCPP_ERROR(rclcpp::get_logger("FoilDynamics"), "Invalid model entity");
    return;
  }

  modelName_ = model_.Name(_ecm);

  // Load link
  if (_sdf->HasElement("link_name"))
  {
    std::string linkName = _sdf->Get<std::string>("link_name");
    auto linkEntity = model_.LinkByName(_ecm, linkName);
    if (linkEntity == gz::sim::kNullEntity)
    {
      RCLCPP_ERROR(rclcpp::get_logger("FoilDynamics"), "Link [%s] not found", linkName.c_str());
    }
    else
    {
      link_ = gz::sim::Link(linkEntity);
    }
  }
  link_.EnableVelocityChecks(_ecm);

  // Parse parameters
  if (_sdf->HasElement("cop"))
    cp_ = _sdf->Get<gz::math::Vector3d>("cop");

  if (_sdf->HasElement("forward"))
    forward_ = _sdf->Get<gz::math::Vector3d>("forward");

  if (_sdf->HasElement("upward"))
    upward_ = _sdf->Get<gz::math::Vector3d>("upward");

  if (_sdf->HasElement("area"))
    area_ = _sdf->Get<double>("area");

  if (_sdf->HasElement("clmax"))
    clmax_ = _sdf->Get<double>("clmax");

  if (_sdf->HasElement("cdmax"))
    cdmax_ = _sdf->Get<double>("cdmax");

  if (_sdf->HasElement("fluid_density"))
    rho_ = _sdf->Get<double>("fluid_density");

  RCLCPP_INFO(rclcpp::get_logger("FoilDynamics"),
              "FoilDynamics loaded for model [%s]", modelName_.c_str());

  node_.Subscribe("/wind", &FoilDynamics::OnWindMsg, this);
}


/////////////////////////////////////////////////
void FoilDynamics::OnWindMsg(const gz::msgs::Vector3d &_msg) {
  wind_ = gz::msgs::Convert(_msg);
}


/////////////////////////////////////////////////
void FoilDynamics::PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!link_.Valid(_ecm))
    return;

  // Get world linear velocity
  auto optVel = link_.WorldLinearVelocity(_ecm);
  if (!optVel)
    return;
  gz::math::Vector3d vel = *optVel;

  // Get world pose
  auto optPose = link_.WorldPose(_ecm);
  if (!optPose)
    return;
  gz::math::Pose3d pose = *optPose;
  
  // Rotate forward/upward vectors to world frame
  gz::math::Vector3d forwardI = pose.Rot().RotateVector(forward_);
  gz::math::Vector3d upwardI = pose.Rot().RotateVector(upward_);
  gz::math::Vector3d ldNormal = forwardI.Cross(upwardI).Normalized();

  // Project velocity into lift–drag plane
  // wind is the base air speed
  gz::math::Vector3d aw = wind_ - vel;
  // velInLDPlane is the "apparent velocity" if we assume that wind is the base air speed
  gz::math::Vector3d velInLDPlane = -aw;

  if (aw.Length() < 0.01)
    return;

  // Drag and lift directions
  gz::math::Vector3d dragDir = -velInLDPlane.Normalized();
  gz::math::Vector3d liftDir = -ldNormal.Cross(velInLDPlane).Normalized();

  // Angle of attack
  double cosAlpha = gz::math::clamp(
      forwardI.Dot(velInLDPlane) / (forwardI.Length() * velInLDPlane.Length()),
      -1.0, 1.0);

  double alphaSign = -upwardI.Dot(velInLDPlane) /
                     (upwardI.Length() * velInLDPlane.Length());

  double alpha = fabs((alphaSign >= 0.0) ? acos(cosAlpha) : -acos(cosAlpha));

  // Dynamic pressure
  double speedInLDPlane = velInLDPlane.Length();
  double q = 0.5 * rho_ * speedInLDPlane * speedInLDPlane;

  // Lift and drag coefficients
  double cl = clmax_ * sin(2 * alpha);
  double cd = cdmax_ / 2 * (1 - cos (2 * alpha));

  // Forces
  gz::math::Vector3d lift = cl * q * area_ * liftDir;
  gz::math::Vector3d drag = cd * q * area_ * dragDir;
  gz::math::Vector3d force = lift + drag;

  RCLCPP_INFO(rclcpp::get_logger("FoilDynamics"), "Velocity: %f %f %f", vel.X(), vel.Y(), vel.Z());
  RCLCPP_INFO(rclcpp::get_logger("FoilDynamics"), "VelocityInLDPlane: %f %f %f", velInLDPlane.X(), velInLDPlane.Y(), velInLDPlane.Z());
  RCLCPP_INFO(rclcpp::get_logger("FoilDynamics"), "Wind Velocity: %f %f %f", wind_.X(), wind_.Y(), wind_.Z());
  RCLCPP_INFO(rclcpp::get_logger("FoilDynamics"), "Angle: %lf", alpha);
  RCLCPP_INFO(rclcpp::get_logger("FoilDynamics"), "LiftDirection: %f %f %f", liftDir.X(), liftDir.Y(), liftDir.Z());
  RCLCPP_INFO(rclcpp::get_logger("FoilDynamics"), "DragDirection: %f %f %f", dragDir.X(), dragDir.Y(), dragDir.Z());
  RCLCPP_INFO(rclcpp::get_logger("FoilDynamics"), "Lift: %f %f %f", lift.X(), lift.Y(), lift.Z());
  RCLCPP_INFO(rclcpp::get_logger("FoilDynamics"), "Drag: %f %f %f", drag.X(), drag.Y(), drag.Z());
  RCLCPP_INFO(rclcpp::get_logger("FoilDynamics"), "Force: %f %f %f\n\n", force.X(), force.Y(), force.Z());

  // Apply world force
  link_.AddWorldForce(_ecm, force);
}

}

/////////////////////////////////////////////////
GZ_ADD_PLUGIN(foil_dynamics::FoilDynamics,
              gz::sim::System,
              foil_dynamics::FoilDynamics::ISystemConfigure,
              foil_dynamics::FoilDynamics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(foil_dynamics::FoilDynamics, "foil_dynamics::FoilDynamics")
