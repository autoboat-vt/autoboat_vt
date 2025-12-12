#include "../include/SailLimits.hh"
#include <cmath>

namespace sail_limits
{

/////////////////////////////////////////////////
SailLimits::SailLimits()
:  limit_(-1.6, 1.6),
   limit2_(-1.6, 1.6)
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
}


/////////////////////////////////////////////////
void SailLimits::OnLimitMsg(const gz::msgs::Double &_msg) {
  double limit = _msg.data()*3.14159/180;
  if (limit < 0.01) {
    limit = 0.01;
  }
  //RCLCPP_INFO(rclcpp::get_logger("SailLimits"), "Limit updated.");
  // this is to make it smoother - it does this in two timesteps instead of one so the sail doesn't snap.
  limit_ = {(limit_.X() - (-limit))/2, (limit_.Y() - limit)/2};
  limit2_ = {-limit, limit};
}

/////////////////////////////////////////////////
void SailLimits::PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!joint_.Valid(_ecm))
    return;
  
  std::vector<gz::math::Vector2d> limitsVector = {limit_};
  joint_.SetPositionLimits(_ecm, limitsVector);
  limit_ = limit2_;
}

}

/////////////////////////////////////////////////
GZ_ADD_PLUGIN(sail_limits::SailLimits,
              gz::sim::System,
              sail_limits::SailLimits::ISystemConfigure,
              sail_limits::SailLimits::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(sail_limits::SailLimits, "sail_limits::SailLimits")
