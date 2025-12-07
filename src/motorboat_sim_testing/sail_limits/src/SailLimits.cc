#include "../include/SailLimits.hh"
#include <cmath>

namespace sail_limits
{

/////////////////////////////////////////////////
SailLimits::SailLimits()
:  limit_(-1.6, 1.6)
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
  }

  RCLCPP_INFO(rclcpp::get_logger("SailLimits"),
              "SailLimits loaded for model [%s]", modelName_.c_str());

  node_.Subscribe("/sail_limit", &SailLimits::OnLimitMsg, this);
}


/////////////////////////////////////////////////
void SailLimits::OnLimitMsg(const gz::msgs::Vector2d &_msg) {
  // TODO (btw u have to convert the message from degrees to radians btw)
}

}

/////////////////////////////////////////////////
GZ_ADD_PLUGIN(sail_limits::SailLimits,
              gz::sim::System,
              sail_limits::SailLimits::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(sail_limits::SailLimits, "sail_limits::SailLimits")
