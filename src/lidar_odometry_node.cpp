#include <rclcpp/rclcpp.hpp>
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

        this->declare_parameters("", LidarOdometry::Params::GetROSDeclaration());

        std::map<std::string, rclcpp::ParameterValue> parameters;
        this->get_parameters("", parameters);

        auto odom_params = LidarOdometry::Params::FromROSParams(parameters);
        std::cout<<odom_params.keyframe_voxel_size<<std::endl;
    }

protected:
    void onPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr)
    {

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
