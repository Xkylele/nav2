#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LivoxToPointCloud2 : public rclcpp::Node
{
public:
    LivoxToPointCloud2() : Node("livox_to_pointcloud2")
    {
        sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 10,
            std::bind(&LivoxToPointCloud2::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/livox/points", 10);

        RCLCPP_INFO(this->get_logger(), "Livox → PointCloud2 node started");
    }

private:
    void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;

        cloud.reserve(msg->point_num);

        for (uint32_t i = 0; i < msg->point_num; ++i)
        {
            pcl::PointXYZI point;

            point.x = msg->points[i].x;
            point.y = msg->points[i].y;
            point.z = msg->points[i].z;
            point.intensity = msg->points[i].reflectivity;

            cloud.push_back(point);
        }

        cloud.width = cloud.size();
        cloud.height = 1;
        cloud.is_dense = false;

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(cloud, output);

        output.header = msg->header;
        output.header.frame_id = "livox_frame"; // ⚠️ 根据你的TF改

        pub_->publish(output);
    }

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxToPointCloud2>());
    rclcpp::shutdown();
    return 0;
}