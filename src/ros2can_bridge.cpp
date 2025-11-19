#include "ros2can_bridge.hpp"

CanBridge::CanBridge()
{
    this->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->sock < 0)
    {
        throw std::runtime_error("failed to open socket");
    }

    int enable_canfd = 1;
    if (setsockopt(this->sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
    &enable_canfd, sizeof(enable_canfd)) < 0)
    {
        close(this->sock);
        throw std::runtime_error("failed to socketopt canfd");
    }

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(this->sock, SIOCGIFINDEX, &ifr) < 0)
    {
        close(this->sock);
        throw std::runtime_error("failed to use ioctl");
    }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(this->sock, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
        close(this->sock);
        throw std::runtime_error("failed to bind");
    }
}

CanBridge::~CanBridge()
{
    close(this->sock);
}

int CanBridge::send_float(int canid, std::vector<float> txdata_f)
{
    int byte_length = (int)txdata_f.size() * 4;
    if (byte_length > 64)
    {
        return -1;
    }

    canfd_frame frame{};
    frame.can_id = canid;
    frame.len = byte_length;
    frame.flags = 1;
    
}