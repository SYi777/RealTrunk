#include "data_broadcast.h"
#include <string.h>
#include <unistd.h>
#include <iostream>

DataBroadcast::DataBroadcast(std::string udp_ip, int udp_port): udp_ip_(udp_ip), udp_port_(udp_port), socket_fd_(-1)
{

}

DataBroadcast::~DataBroadcast()
{
    if (socket_fd_ != -1)
    {
        close(socket_fd_);
    }
}

bool DataBroadcast::Init()
{
    // 设置网络通信
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0)
    {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }

    // 启用广播选项
    int broadcastEnable = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0)
    {
        std::cerr << "Failed to enable broadcast option" << std::endl;
        close(socket_fd_);
        return false;
    }
    
    memset(&broadcast_addr_, 0, sizeof(broadcast_addr_));
    broadcast_addr_.sin_family = AF_INET;
    broadcast_addr_.sin_port = htons(udp_port_);                     // 修改为目标端口
    broadcast_addr_.sin_addr.s_addr = inet_addr(udp_ip_.c_str()); // 广播地址
    // inet_pton(AF_INET, "192.168.10.33", &broadcastAddr.sin_addr); // 修改为目标IP地址
    return true;
}

void DataBroadcast::CloseSocket()
{
    if (socket_fd_ != -1)
    {
        close(socket_fd_);
    }
}

// 通过网络发送数据
void DataBroadcast::SendNetworkData(const std::string &data)
{
    sendto(socket_fd_, data.c_str(), data.size(), 0, (const struct sockaddr *)&broadcast_addr_, sizeof(broadcast_addr_));
}

// 通过网络发送十六进制数据
void DataBroadcast::SendNetworkHexData(const std::vector<uint8_t> &data)
{
    sendto(socket_fd_, data.data(), data.size(), 0, (const struct sockaddr *)&broadcast_addr_, sizeof(broadcast_addr_));
}