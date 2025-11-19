#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <vector>
#include <stdexcept>
#include <cstring>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class CanBridge
{
public:
    CanBridge(const char* Ifname);
    ~CanBridge();
    int send_float(int canid, std::vector<float> txdata_f);
private:
    const char* ifname;
    int sock;
    union Data
    {
    uint32_t data_ui32;
    float data_f32;
    };
};

class ros2can_bridge_node
: public rclcpp_lifecycle::LifecycleNode
{
public:
    ros2can_bridge_node();
private:
    const char* ifname;
};