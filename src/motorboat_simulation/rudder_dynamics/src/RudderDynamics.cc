#include "../include/RudderDynamics.hh"
#include <cmath>

namespace rudder_dynamics
{

/////////////////////////////////////////////////
RudderDynamics::RudderDynamics()
: rho_(1000.1),
  cp_(0, 0, 0),
  forward_(1, 0, 0),
  upward_(0, 0, 1),
  area_(1.0),
  clmax_(1.5),
  cdmax_(1.0)
{
  RCLCPP_INFO(rclcpp::get_logger("RudderDynamics"), "RudderDynamics plugin constructed");
}

/////////////////////////////////////////////////
RudderDynamics::~RudderDynamics() = default;

/////////////////////////////////////////////////
void RudderDynamics::Configure(const gz::sim::Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             gz::sim::EntityComponentManager &_ecm,
                             gz::sim::EventManager &)
{
  model_ = gz::sim::Model(_entity);
  if (!model_.Valid(_ecm))
  {
    RCLCPP_ERROR(rclcpp::get_logger("RudderDynamics"), "Invalid model entity");
    return;
  }

  modelName_ = model_.Name(_ecm);

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

  // Load links
  std::vector<std::string> linkNames = {};
  for (int i = 0; i < 200; i++) {
    if (_sdf->HasElement("link" + std::to_string(i)))
      linkNames.push_back(_sdf->Get<std::string>("link" + std::to_string(i)));
    else
      break;
  }
  
  links_ = {};
  for (std::string linkName : linkNames) {
    auto linkEntity = model_.LinkByName(_ecm, linkName);
    if (linkEntity == gz::sim::kNullEntity)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RudderDynamics"), "Link [%s] not found", linkName.c_str());
    }
    else
    {
      links_.emplace_back(gz::sim::Link(linkEntity));
      links_.back().EnableVelocityChecks(_ecm);
      RCLCPP_INFO(rclcpp::get_logger("RudderDynamics"), "RudderDynamics loaded for link [%s]", linkName.c_str());
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RudderDynamics"),
              "RudderDynamics loaded for model [%s]", modelName_.c_str());
}


/////////////////////////////////////////////////
void RudderDynamics::PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // Get world linear velocity
  auto optVel = links_[0].WorldLinearVelocity(_ecm);
  if (!optVel)
    return;
  gz::math::Vector3d vel = *optVel;

  // Get world pose
  auto optPose = links_[0].WorldPose(_ecm);
  if (!optPose)
    return;
  gz::math::Pose3d pose = *optPose;
  
  // Rotate forward/upward vectors to world frame
  gz::math::Vector3d forwardI = pose.Rot().RotateVector(forward_);
  gz::math::Vector3d upwardI = pose.Rot().RotateVector(upward_);
  gz::math::Vector3d ldNormal = forwardI.Cross(upwardI).Normalized();

  // Project velocity into lift–drag plane
  gz::math::Vector3d velInLDPlane = vel - vel.Dot(ldNormal)*ldNormal;

  if (velInLDPlane.Length() < 0.01)
    return;

  // Drag and lift directions
  gz::math::Vector3d dragDir = -velInLDPlane.Normalized();
  gz::math::Vector3d liftDir = -ldNormal.Cross(velInLDPlane).Normalized();

  // Angle of attack
  double cosAlpha = gz::math::clamp(
      forwardI.Dot(velInLDPlane) / (forwardI.Length() * velInLDPlane.Length()),
      -1.0, 1.0);

  double alphaSign = upwardI.Dot(velInLDPlane) /
                     (upwardI.Length() * velInLDPlane.Length());

  double alpha = (alphaSign >= 0.0) ? acos(cosAlpha) : -acos(cosAlpha);

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

  // RCLCPP_INFO(rclcpp::get_logger("RudderDynamics"), "Velocity: %f %f %f", vel.X(), vel.Y(), vel.Z());
  // RCLCPP_INFO(rclcpp::get_logger("RudderDynamics"), "VelocityInLDPlane: %f %f %f", velInLDPlane.X(), velInLDPlane.Y(), velInLDPlane.Z());
  // RCLCPP_INFO(rclcpp::get_logger("RudderDynamics"), "Angle: %lf", alpha);
  // RCLCPP_INFO(rclcpp::get_logger("RudderDynamics"), "LiftDirection: %f %f %f", liftDir.X(), liftDir.Y(), liftDir.Z());
  // RCLCPP_INFO(rclcpp::get_logger("RudderDynamics"), "DragDirection: %f %f %f", dragDir.X(), dragDir.Y(), dragDir.Z());
  // RCLCPP_INFO(rclcpp::get_logger("RudderDynamics"), "Lift: %f %f %f", lift.X(), lift.Y(), lift.Z());
  // RCLCPP_INFO(rclcpp::get_logger("RudderDynamics"), "Drag: %f %f %f", drag.X(), drag.Y(), drag.Z());
  // RCLCPP_INFO(rclcpp::get_logger("RudderDynamics"), "Force: %f %f %f\n\n", force.X(), force.Y(), force.Z());

  // Apply world force
  for (gz::sim::Link link : links_) {
    if (link.Valid(_ecm))
      link.AddWorldForce(_ecm, force);
    else
      RCLCPP_INFO(rclcpp::get_logger("RudderDynamics"), "link failed to apply rudder.");
  }
}

}

/////////////////////////////////////////////////
GZ_ADD_PLUGIN(rudder_dynamics::RudderDynamics,
              gz::sim::System,
              rudder_dynamics::RudderDynamics::ISystemConfigure,
              rudder_dynamics::RudderDynamics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(rudder_dynamics::RudderDynamics, "rudder_dynamics::RudderDynamics")
