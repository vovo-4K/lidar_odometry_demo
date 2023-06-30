#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

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

        auto keyframe_cloud_ptr = odometry_ptr_->getKeyFrameCloud();
        sensor_msgs::msg::PointCloud2 keyframe_cloud_msg;
        pcl::toROSMsg(*keyframe_cloud_ptr, keyframe_cloud_msg);
        keyframe_cloud_msg.header.stamp = msg_ptr->header.stamp;
        keyframe_cloud_msg.header.frame_id = "/base_scan";
        keyframe_publisher_->publish(keyframe_cloud_msg);

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_publisher_;

    std::shared_ptr<LidarOdometry> odometry_ptr_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
