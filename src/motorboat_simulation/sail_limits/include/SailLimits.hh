#ifndef SAIL_LIMITS_HH
#define SAIL_LIMITS_HH

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/Utility.hh>

#include <gz/math/Vector2.hh>

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace sail_limits
{
  class SailLimits
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
  public:
    SailLimits();
    ~SailLimits() override;

    // Called once when the plugin is loaded
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr) override;

    // Called every time the limit topic is published to.
    void OnLimitMsg(const gz::msgs::Double &_msg);

    // Called every simulation iteration
    void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

  private:
    /// \brief The model entity
    gz::sim::Model model_{gz::sim::kNullEntity};

    /// \brief The model name (for logging)
    std::string modelName_;

    /// \brief The rotation joint
    gz::sim::Joint joint_{gz::sim::kNullEntity};

    /// \brief Current joint posiiton
    double currentPosition_;

    /// \brief Jint limits
    std::vector<gz::math::Vector2d> limit_{{-1.6, 1.6}, {-1.6, 1.6}, {-1.6, 1.6}, {-1.6, 1.6}, {-1.6, 1.6}, {-1.6, 1.6}, {-1.6, 1.6}, {-1.6, 1.6}};

    /// \brief Gazebo communication node.
    gz::transport::Node node_;
  };
}

#endif
