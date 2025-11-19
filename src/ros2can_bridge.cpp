#include "ros2can_bridge.hpp"

CanBridge::CanBridge()
{
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0)
    {
        throw std::runtime_error("failed to open socket");
    }

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
    {
        close(sock);
        throw std::runtime_error("failed to use ioctl");
    }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
        close(sock);
        throw std::runtime_error("failed to bind");
    }
}