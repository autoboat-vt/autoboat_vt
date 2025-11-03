#ifndef FOIL_DYNAMICS_HH
#define FOIL_DYNAMICS_HH

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/plugin/Register.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace foil_dynamics
{
  class FoilDynamics
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemUpdate
  {
  public:
    FoilDynamics();
    ~FoilDynamics() override;

    // Called once when the plugin is loaded
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr) override;

    // Called every simulation iteration
    void Update(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

  private:
    /// \brief The model entity
    gz::sim::Model model_{gz::sim::kNullEntity};

    /// \brief The model name (for logging)
    std::string modelName_;

    /// \brief The hydrodynamic link
    gz::sim::Link link_{gz::sim::kNullEntity};

    /// \brief Fluid density [kg/m^3]
    double rho_{1000.1};

    /// \brief Center of pressure offset in link frame
    gz::math::Vector3d cp_{0, 0, 0};

    /// \brief Forward direction in link frame
    gz::math::Vector3d forward_{1, 0, 0};

    /// \brief Upward direction in link frame
    gz::math::Vector3d upward_{0, 0, 1};

    /// \brief Reference area [m^2]
    double area_{1.0};

    /// \brief Lift multiplier
    double mult_lift_{1.0};

    /// \brief Drag multiplier
    double mult_drag_{1.0};
  };
} // namespace foil_dynamics

#endif
