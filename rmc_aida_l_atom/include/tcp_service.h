#include <iostream>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h>  // 包含 inet_addr 所需的头文件
#include <cstring>  // 用于使用 strlen 函数


// TCP服务端
class Tcp_Service
{
private:
    int server_fd;
    static bool *thread_exist;
    pthread_t listen_thread;
    // 监听和连接处理线程
    static void *listen_and_accept(void *arg);
    // 线程处理函数
    static void *handle_client(void *arg);
public:
    Tcp_Service(int port, std::string ip);
    ~Tcp_Service();
    
    // 开启服务端线程
    bool start_thread();
    // 是否读取到信号
    static bool *ros_ctrl;
};