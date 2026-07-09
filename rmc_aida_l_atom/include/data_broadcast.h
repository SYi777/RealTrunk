#ifndef DATABROADCAST_H
#define DATABROADCAST_H

#include <arpa/inet.h>
#include <string>
#include <vector>

class DataBroadcast
{
private:
    int socket_fd_;
    int udp_port_;
    std::string udp_ip_;
    sockaddr_in broadcast_addr_;
public:
    DataBroadcast(std::string udp_ip, int udp_port);
    ~DataBroadcast();
    bool Init();
    void CloseSocket();
    // 通过网络发送数据
    void SendNetworkData(const std::string &data);
    // 通过网络发送十六进制数据
    void SendNetworkHexData(const std::vector<uint8_t> &data);
};


#endif DATABROADCAST_H