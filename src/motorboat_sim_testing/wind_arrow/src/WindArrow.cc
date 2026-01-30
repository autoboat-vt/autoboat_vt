#include "../include/WindArrow.hh"
#include <cmath>

namespace wind_arrow
{

/////////////////////////////////////////////////
WindArrow::WindArrow()
:  wind_(0, 0, 0)
{
  RCLCPP_INFO(rclcpp::get_logger("WindArrow"), "WindArrow plugin constructed");
}

/////////////////////////////////////////////////
WindArrow::~WindArrow() = default;

/////////////////////////////////////////////////
void WindArrow::Configure(const gz::sim::Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             gz::sim::EntityComponentManager &_ecm,
                             gz::sim::EventManager &)
{
  model_ = gz::sim::Model(_entity);

  if (!model_.Valid(_ecm))
  {
    RCLCPP_ERROR(rclcpp::get_logger("WindArrow"), "Invalid model entity");
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
      RCLCPP_ERROR(rclcpp::get_logger("WindArrow"), "Joint [%s] not found", jointName.c_str());
    }
    else
    {
      joint_ = gz::sim::Joint(jointEntity);
    }
    RCLCPP_INFO(rclcpp::get_logger("WindArrow"), "WindArrow loaded for joint [%s]", jointName.c_str());
  }

  // Load link
  if (_sdf->HasElement("base_link_name"))
  {
    std::string linkName = _sdf->Get<std::string>("base_link_name");
    auto linkEntity = model_.LinkByName(_ecm, linkName);
    if (linkEntity == gz::sim::kNullEntity)
    {
      RCLCPP_ERROR(rclcpp::get_logger("WindArrow"), "Link [%s] not found", linkName.c_str());
    }
    else
    {
      base_link_ = gz::sim::Link(linkEntity);
    }
    RCLCPP_INFO(rclcpp::get_logger("WindArrow"), "WindArrow loaded for link [%s]", linkName.c_str());
  }
  
  node_.Subscribe("/wind", &WindArrow::OnWindMsg, this);
}


/////////////////////////////////////////////////
void WindArrow::OnWindMsg(const gz::msgs::Vector3d &_msg) {
  wind_ = gz::msgs::Convert(_msg);
  
}

/////////////////////////////////////////////////
void WindArrow::PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!model_.Valid(_ecm))
    return;
  
  if (wind_.Length() == 0)
    return;
  
  auto pose = base_link_.WorldPose(_ecm);
  if (!pose)
    return;
  
  double yaw = (*pose).Yaw();
  
  // I can't directly control the position, but I can set the limit!
  double limit = acos(gz::math::clamp(wind_.X()/wind_.Length(), -1.0, 1.0));
  if (wind_.Y() < 0)
    limit = -limit;

  std::vector<gz::math::Vector2d> limitsVector = {{limit-yaw, limit-yaw+0.05}};
  joint_.SetPositionLimits(_ecm, limitsVector);
}

}

/////////////////////////////////////////////////
GZ_ADD_PLUGIN(wind_arrow::WindArrow,
              gz::sim::System,
              wind_arrow::WindArrow::ISystemConfigure,
              wind_arrow::WindArrow::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(wind_arrow::WindArrow, "wind_arrow::WindArrow")
