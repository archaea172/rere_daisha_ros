#include "ros2can_bridge.hpp"

CanBridge::CanBridge()
{
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0)
    {
        throw std::runtime_error("failed to open socket");
    }

    
}