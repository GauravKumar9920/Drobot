#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
#include "apriltag_dock/TfPlugin.hh"

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

GZ_ADD_PLUGIN(
    apriltag_dock_plugins::TfPublisherPlugin,
    gz::sim::System,
    apriltag_dock_plugins::TfPublisherPlugin::ISystemConfigure,
    apriltag_dock_plugins::TfPublisherPlugin::ISystemPreUpdate,
    apriltag_dock_plugins::TfPublisherPlugin::ISystemPostUpdate)

namespace apriltag_dock_plugins
{

    struct TfPublisherPlugin::PrivateData
    {
        std::shared_ptr<rclcpp::Node> rosNode;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
        std::string parentFrame;
        std::string childFrame;
        std::string linkName;
        gz::sim::Entity entity;
    };

    TfPublisherPlugin::TfPublisherPlugin() : dataPtr(std::make_unique<PrivateData>()) {}

    void TfPublisherPlugin::Configure(
        const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventManager)
    {
        // Initialize ROS 2 node
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
        this->dataPtr->rosNode = std::make_shared<rclcpp::Node>("tf_publisher_plugin");
        this->dataPtr->tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this->dataPtr->rosNode);

        // Read parameters from SDF
        if (!_sdf->HasElement("parent_frame"))
        {
            gzerr << "Missing required parameter <parent_frame>, defaulting to 'chassis'.\n";
            this->dataPtr->parentFrame = "chassis";
        }
        else
        {
            this->dataPtr->parentFrame = _sdf->Get<std::string>("parent_frame");
        }

        if (!_sdf->HasElement("child_frame"))
        {
            gzerr << "Missing required parameter <child_frame>, defaulting to 'camera_link'.\n";
            this->dataPtr->childFrame = "camera_link";
        }
        else
        {
            this->dataPtr->childFrame = _sdf->Get<std::string>("child_frame");
        }

        if (!_sdf->HasElement("link_name"))
        {
            gzerr << "Missing required parameter <link_name>, plugin will not function.\n";
            return;
        }
        this->dataPtr->linkName = _sdf->Get<std::string>("link_name");

        // Store the entity
        this->dataPtr->entity = _entity;

        gzmsg << "TfPublisherPlugin configured for parent_frame: " << this->dataPtr->parentFrame
              << ", child_frame: " << this->dataPtr->childFrame
              << ", link_name: " << this->dataPtr->linkName << "\n";
    }

    void TfPublisherPlugin::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
    {
        // Get link entity
        auto linkEntity = _ecm.EntityByComponents(
            gz::sim::components::Link(), gz::sim::components::Name(this->dataPtr->linkName));

        if (linkEntity == gz::sim::kNullEntity)
        {
            gzerr << "Link with name [" << this->dataPtr->linkName << "] not found. Skipping update.\n";
            return;
        }

        // Get the pose of the link
        const auto *poseComp = _ecm.Component<gz::sim::components::Pose>(linkEntity);
        if (!poseComp)
        {
            gzerr << "Pose component not found for link [" << this->dataPtr->linkName << "].\n";
            return;
        }

        auto pose = poseComp->Data();

        // Publish the transform
        geometry_msgs::msg::TransformStamped tfMsg;
        tfMsg.header.stamp = this->dataPtr->rosNode->get_clock()->now();
        tfMsg.header.frame_id = this->dataPtr->parentFrame;
        tfMsg.child_frame_id = this->dataPtr->childFrame;

        tfMsg.transform.translation.x = pose.Pos().X();
        tfMsg.transform.translation.y = pose.Pos().Y();
        tfMsg.transform.translation.z = pose.Pos().Z();

        tfMsg.transform.rotation.x = pose.Rot().X();
        tfMsg.transform.rotation.y = pose.Rot().Y();
        tfMsg.transform.rotation.z = pose.Rot().Z();
        tfMsg.transform.rotation.w = pose.Rot().W();

        this->dataPtr->tfBroadcaster->sendTransform(tfMsg);
    }

    void TfPublisherPlugin::PostUpdate(
        const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm)
    {
        if (!_info.paused)
        {
            rclcpp::spin_some(this->ros_node_);
        }
    }

} // namespace apriltag_dock_plugins