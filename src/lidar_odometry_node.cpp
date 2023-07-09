#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "lidar_odometry.h"

class LidarOdometryNode : public rclcpp::Node
{
public:
    LidarOdometryNode()
            : Node("lidar_odometry_node")
    {
        cloud_subscription_ =
                this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar_points", rclcpp::QoS(10),
                                                                         std::bind(&LidarOdometryNode::onPointCloudCallback, this, std::placeholders::_1));

        keyframe_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/keyframe_cloud", rclcpp::QoS(1).keep_last(1));
        deskewed_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/deskewed_cloud", rclcpp::QoS(10));

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", rclcpp::QoS(10));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        this->declare_parameters("lidar_odometry", LidarOdometry::Params::GetROSDeclaration());

        std::map<std::string, rclcpp::ParameterValue> parameters;
        this->get_parameters("lidar_odometry", parameters);

        auto odom_params = LidarOdometry::Params::FromROS2Params(parameters);

        odometry_ptr_ = std::make_shared<LidarOdometry>(odom_params);
    }

protected:
    void onPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr)
    {
        pcl::PointCloud<lidar_point::PointXYZIRT> cloud;
        pcl::fromROSMsg(*msg_ptr, cloud);

        odometry_ptr_->processCloud(cloud);

        if (keyframe_publisher_->get_subscription_count()>0) {
            auto keyframe_cloud_ptr = odometry_ptr_->getKeyFrameCloud();
            sensor_msgs::msg::PointCloud2 keyframe_cloud_msg;
            pcl::toROSMsg(*keyframe_cloud_ptr, keyframe_cloud_msg);
            keyframe_cloud_msg.header.stamp = msg_ptr->header.stamp;
            keyframe_cloud_msg.header.frame_id = "odom";
            keyframe_publisher_->publish(keyframe_cloud_msg);
        }

        if (deskewed_publisher_->get_subscription_count()>0) {
            auto deskewed_cloud_ptr = odometry_ptr_->getTempCloud();
            if (deskewed_cloud_ptr) {
                sensor_msgs::msg::PointCloud2 deskewed_cloud_msg;
                pcl::toROSMsg(*deskewed_cloud_ptr, deskewed_cloud_msg);
                deskewed_cloud_msg.header.stamp = msg_ptr->header.stamp;
                deskewed_cloud_msg.header.frame_id = "base_scan";
                deskewed_publisher_->publish(deskewed_cloud_msg);
            }
        }
        auto pose = odometry_ptr_->getCurrentPose();
        // publish TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = msg_ptr->header.stamp;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_scan";

        t.transform.translation.x = pose.translation.x();
        t.transform.translation.y = pose.translation.y();
        t.transform.translation.z = pose.translation.z();

        t.transform.rotation.w = pose.rotation.w();
        t.transform.rotation.x = pose.rotation.x();
        t.transform.rotation.y = pose.rotation.y();
        t.transform.rotation.z = pose.rotation.z();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);

        // publish odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg_ptr->header.stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_scan";
        odom_msg.pose.pose.position.x = pose.translation.x();
        odom_msg.pose.pose.position.y = pose.translation.y();
        odom_msg.pose.pose.position.z = pose.translation.z();
        odom_msg.pose.pose.orientation.w = pose.rotation.w();
        odom_msg.pose.pose.orientation.x = pose.rotation.x();
        odom_msg.pose.pose.orientation.y = pose.rotation.y();
        odom_msg.pose.pose.orientation.z = pose.rotation.z();
        odom_publisher_->publish(odom_msg);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<LidarOdometry> odometry_ptr_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
