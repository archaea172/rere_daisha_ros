#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"

class CanBridge
{
public:
    CanBridge();
private:
    const char* ifname;
};

class ros2can_bridge_node
: rclcpp::Node
{
public:
    ros2can_bridge_node();
private:
    const char* ifname;
};