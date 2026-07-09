#include "serial_port.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
// #include <sys/ioctl.h>
// #include <sys/select.h>


// 初始化波特率映射表
const std::map<unsigned int, unsigned int> SerialPort::baud_rate_map = {
    {0, B0},
    {50, B50},
    {75, B75},
    {110, B110},
    {134, B134},
    {150, B150},
    {200, B200},
    {300, B300},
    {600, B600},
    {1200, B1200},
    {1800, B1800},
    {2400, B2400},
    {4800, B4800},
    {9600, B9600},
    {19200, B19200},
    {38400, B38400},
    {57600, B57600},
    {115200, B115200},
    {230400, B230400},
    {460800, B460800},
    {500000, B500000},
    {576000, B576000},
    {921600, B921600},
    {1000000, B1000000},
    {1152000, B1152000},
    {1500000, B1500000},
    {2000000, B2000000},
    {2500000, B2500000},
    {3000000, B3000000},
    {3500000, B3500000},
    {4000000, B4000000}
};

SerialPort::SerialPort(const std::string &port_name, unsigned int baud_rate): port_name_(port_name), fd_(-1), baud_rate_(GetBaudRateConstant(baud_rate))
{

}

// 打开串口设备并配置串口参数
bool SerialPort::OpenPort()
{
    int timeout_ms = 2;
    fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1)
    {
        std::cerr << "Error: Unable to open serial port: " << std::endl;
        return false;
    }
    else
    {
        // 设置为正常模式，不是非阻塞模式
        fcntl(fd_, F_SETFL, 0);
        std::cout << "Serial port opened successfully." << std::endl;
    }
    struct termios options;

    if (tcgetattr(fd_, &options) != 0)
    {
        std::cerr << "Error: Unable to get serial port attributes." << std::endl;
        ClosePort();
        return false;
    }

    // 设置波特率
    cfsetispeed(&options, baud_rate_);
    cfsetospeed(&options, baud_rate_);

    // 设置数据位, 停止位和校验位
    options.c_cflag &= ~PARENB; // 无校验位
    options.c_cflag &= ~CSTOPB; // 1个停止位
    options.c_cflag &= ~CSIZE;  // 清除数据位
    options.c_cflag |= CS8;     // 8个数据位

    // 设置串口模式
    options.c_cflag |= CLOCAL; // 本地连接, 不需要控制线
    options.c_cflag |= CREAD;  // 启用接收功能

    // 设置输入输出模式
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // 禁用软件流控
    options.c_iflag &= ~ICRNL;                          // 禁用换行符转换
    options.c_oflag &= ~OPOST;                          // 禁用输出处理
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 禁用行模式

    // 清除等待队列
    tcflush(fd_, TCIFLUSH);

    // 设置新的串口参数
    if (tcsetattr(fd_, TCSANOW, &options) != 0)
    {
        std::cerr << "Error: Unable to set serial port attributes." << std::endl;
        ClosePort();
        return false;
    }

    std::cout << "Serial port configured successfully." << std::endl;

    // fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    // if (fd_ == -1)
    // {
    //     std::cerr << "Error: Unable to open serial port." << std::endl;
    //     return false;
    // }
    // else
    // {
    //     fcntl(fd_, F_SETFL, 0);
    //     std::cout << "Serial port opened successfully." << std::endl;
    // }
    // struct termios options;

    // if (tcgetattr(fd_, &options) != 0)
    // {
    //     std::cerr << "Error: Unable to get serial port attributes." << std::endl;
    //     ClosePort();
    //     return false;
    // }

    // cfsetispeed(&options, baud_rate_);
    // cfsetospeed(&options, baud_rate_);

    // options.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode
    // options.c_cflag &= ~PARENB;          // No parity bit
    // options.c_cflag &= ~CSTOPB;          // 1 stop bit
    // options.c_cflag &= ~CSIZE;           // Mask the character size bits
    // options.c_cflag |= CS8;              // 8 data bits

    // options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input mode
    // options.c_iflag &= ~(IXON | IXOFF | IXANY);         // Disable software flow control
    // options.c_oflag &= ~OPOST;                          // Raw output mode
    // options.c_iflag &= ~ICRNL;                          // 禁用换行符转换

    // tcflush(fd_, TCIFLUSH);

    // if (tcsetattr(fd_, TCSANOW, &options) != 0)
    // {
    //     std::cerr << "Error: Unable to set serial port attributes." << std::endl;
    //     ClosePort();
    //     return false;
    // }

    // std::cout << "Serial port configured successfully." << std::endl;

    return true;
}

// 关闭串口设备
void SerialPort::ClosePort()
{
    if (fd_ != -1)
    {
        close(fd_);
    }
    std::cout << "Serial port closed." << std::endl;
}

// 向串口发送数据
ssize_t SerialPort::SendData(const std::vector<uint8_t> &data)
{
    ssize_t written = write(fd_, data.data(), data.size());
    if (written < 0)
    {
        std::cerr << "Write failed" << std::endl;
    }
    return written;
}

// 从串口接收数据
std::vector<uint8_t> SerialPort::ReceiveData()
{
    int timeout_ms = 2;
    std::vector<uint8_t> buffer;

       fcntl(fd_, F_SETFL, 0);

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    // 设置超时时间
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;          // 秒
    tv.tv_usec = (timeout_ms % 1000) * 1000; // 微秒

    // 使用select等待文件描述符可读
    int select_ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (select_ret == -1) {
        // select错误
        perror("select error");
        return buffer;
    } else if (select_ret == 0) {
        
        // 超时
        return buffer;
    }
    buffer.resize(512);
    int n = read(fd_, buffer.data(), buffer.size());
    if (n > 0)
    {
        buffer.resize(n);
    }
    else
    {
        buffer.clear();
    }
    return buffer;
}

int SerialPort::GetFd() const
{
    return fd_;
}

unsigned int SerialPort::GetBaudRateConstant(unsigned int baud_rate)
{
    auto it = baud_rate_map.find(baud_rate);
    if (it != baud_rate_map.end())
    {
        return it->second;
    }
    else
    {
        std::cerr << "Unsupported baud rate: " << baud_rate << std::endl;
        return B115200; // 默认波特率
    }
    
    
}