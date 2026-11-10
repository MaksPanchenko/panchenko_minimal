#include "multiwii/multiwii_node.hpp"

#include <exception>
#include <memory>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        auto node = std::make_shared<MultiWiiNode>();
        node->setup();
        rclcpp::spin(node);
    } catch(const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("multiwii"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
