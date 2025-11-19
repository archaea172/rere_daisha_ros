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
    ~CanBridge();
    int send_float(int canid, std::vector<float> txdata_f);
private:
    const char* ifname;
    int sock;
};

class ros2can_bridge_node
: rclcpp::Node
{
public:
    ros2can_bridge_node();
private:
    const char* ifname;
};