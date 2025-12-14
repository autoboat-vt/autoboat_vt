#include "../include/SailLimits.hh"
#include <cmath>

namespace sail_limits
{

/////////////////////////////////////////////////
SailLimits::SailLimits()
{
  RCLCPP_INFO(rclcpp::get_logger("SailLimits"), "SailLimits plugin constructed");
}

/////////////////////////////////////////////////
SailLimits::~SailLimits() = default;

/////////////////////////////////////////////////
void SailLimits::Configure(const gz::sim::Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             gz::sim::EntityComponentManager &_ecm,
                             gz::sim::EventManager &)
{
  model_ = gz::sim::Model(_entity);

  if (!model_.Valid(_ecm))
  {
    RCLCPP_ERROR(rclcpp::get_logger("SailLimits"), "Invalid model entity");
    return;
  }

  modelName_ = model_.Name(_ecm);

  // Load joint
  if (_sdf->HasElement("joint_name"))
  {
    std::string jointName = _sdf->Get<std::string>("joint_name");
    auto jointEntity = model_.JointByName(_ecm, jointName);
    if (jointEntity == gz::sim::kNullEntity)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SailLimits"), "Joint [%s] not found", jointName.c_str());
    }
    else
    {
      joint_ = gz::sim::Joint(jointEntity);
    }

    RCLCPP_INFO(rclcpp::get_logger("SailLimits"), "SailLimits loaded for joint [%s]", jointName.c_str());
  }

  node_.Subscribe("/sail_limit", &SailLimits::OnLimitMsg, this);

  joint_.EnablePositionCheck(_ecm);

  currentPosition_ = 0;
}


/////////////////////////////////////////////////
void SailLimits::OnLimitMsg(const gz::msgs::Double &_msg) {
  double limit = _msg.data()*3.14159/180;
  if (limit < 0.01) {
    limit = 0.01;
  }
  //RCLCPP_INFO(rclcpp::get_logger("SailLimits"), "Limit updated.");
  // this is to make it smoother - it does this in two timesteps instead of one so the sail doesn't snap.
  gz::math::Vector2d currentLimit = limit_[0];
  gz::math::Vector2d newLimit = {-limit, limit};
  if (fabs(currentPosition_) > limit) {
    if (currentPosition_ < 0)
      newLimit = {-limit, -limit+0.01};
    else
      newLimit = {limit, limit+0.01};
  }
  
  limit_ = {newLimit, newLimit, newLimit, newLimit, newLimit, newLimit, newLimit, {-limit, limit}};
}

/////////////////////////////////////////////////
void SailLimits::PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!joint_.Valid(_ecm))
    return;
  
  std::vector<gz::math::Vector2d> limitsVector = {limit_[0]};
  joint_.SetPositionLimits(_ecm, limitsVector);
  for (int i = 0; i < limit_.size()-1; i++) {
    limit_[i] = limit_[i+1];
  }

  auto possiblePos = joint_.Position(_ecm);
  if (possiblePos) {
    currentPosition_ = (*possiblePos)[0];
  }
}

}

/////////////////////////////////////////////////
GZ_ADD_PLUGIN(sail_limits::SailLimits,
              gz::sim::System,
              sail_limits::SailLimits::ISystemConfigure,
              sail_limits::SailLimits::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(sail_limits::SailLimits, "sail_limits::SailLimits")
