#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>
#include <vector>
#include <map>

class SerialPort
{
private:
    int fd_;
    unsigned int baud_rate_;
    std::string port_name_;
    static const std::map<unsigned int, unsigned int> baud_rate_map;

public:
    SerialPort(const std::string &port_name, unsigned int baud_rate);
    bool OpenPort();
    void ClosePort();
    ssize_t SendData(const std::vector<uint8_t> &data);
    std::vector<uint8_t> ReceiveData();
    int GetFd() const;

    static unsigned int GetBaudRateConstant(unsigned int baud_rate);
};


#endif SERIALPORT_H