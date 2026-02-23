#ifndef WIND_ARROW_HH
#define WIND_ARROW_HH

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/Utility.hh>

#include <gz/math/Vector2.hh>

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace wind_arrow
{
  class WindArrow
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
  public:
    WindArrow();
    ~WindArrow() override;

    // Called once when the plugin is loaded
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr) override;

    // Called every time the wind topic is published to.
    void OnWindMsg(const gz::msgs::Vector3d &_msg);

    // Called every simulation iteration
    void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

  private:
    /// \brief The model entity
    gz::sim::Model model_{gz::sim::kNullEntity};

    /// \brief The model name (for logging)
    std::string modelName_;

    /// \brief Wind vector
    gz::math::Vector3d wind_{0, 0, 0};

    /// \brief The arrow joint
    gz::sim::Joint joint_{gz::sim::kNullEntity};

    /// \brief The ship link
    gz::sim::Link base_link_{gz::sim::kNullEntity};

    /// \brief Gazebo communication node.
    gz::transport::Node node_;
  };
}

#endif
