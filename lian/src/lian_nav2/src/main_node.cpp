#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/OccupancyGrid.hpp"
#include "map.cpp"
using namespace std::chrono_literals;

class Lian_node : public rclcpp::Node
{
public:
    Lian_node() : Node("lian_node")
    {
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&Map::getMap, this, std::placeholders::_1));
    }
    
private:
};
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto lian_node = std::make_shared<Lian_node>();

    rclcpp::spin(lian_node);

    rclcpp::shutdown();
}