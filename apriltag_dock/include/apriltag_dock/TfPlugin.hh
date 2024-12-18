#ifndef APRILTAG_DOCK__TFPLUGIN_HH_
#define APRILTAG_DOCK__TFPLUGIN_HH_

#include <math.h>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>

namespace apriltag_dock_plugins
{
    class TfPublisherPlugin : public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPreUpdate,
                              public gz::sim::ISystemPostUpdate
    {
    public:
        TfPublisherPlugin();
        ~TfPublisherPlugin() override = default;

        void Configure(
            const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
            gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventManager);

        void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm);

        void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm);

    private:
        std::shared_ptr<rclcpp::Node> ros_node_;

        struct PrivateData;
        std::unique_ptr<PrivateData> dataPtr;
    };
} // namespace apriltag_dock_plugins

#endif // APRILTAG_DOCK__TFPLUGIN_HH_