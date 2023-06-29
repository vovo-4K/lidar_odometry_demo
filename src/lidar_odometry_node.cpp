#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LidarOdometryNode : public rclcpp::Node
{
public:
    LidarOdometryNode()
            : Node("lidar_odometry_node")
    {
        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar_points", rclcpp::QoS(10),
                                                                                       std::bind(&LidarOdometryNode::onPointCloudCallback, this, std::placeholders::_1));
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
