#include "tcp_service.h"

// 定义静态成员变量
bool *Tcp_Service::thread_exist = new bool(true);
bool *Tcp_Service::ros_ctrl = new bool(false);

Tcp_Service::Tcp_Service(int port, std::string ip)
{
    // 初始化线程保持变量
    *this->thread_exist = true;
    *this->ros_ctrl = false;

    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // 创建 socket 文件描述符
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // 设置 socket 选项
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    // 绑定 socket 到指定的固定 IP 地址和端口
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(ip.c_str());  // 使用 inet_addr 函数
    address.sin_port = htons(port);
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // 监听 socket
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    std::cout << "Server is listening on IP " << port << " and port " << ip << "..." << std::endl;
}

Tcp_Service::~Tcp_Service()
{
    *this->thread_exist = false;
    std::cout << "The TCP server is closed!";
    free(this->ros_ctrl);
    free(this->thread_exist);
}

void *Tcp_Service::listen_and_accept(void *arg)
{
    int server_fd = *((int *)arg);
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    std::cout << "Listening thread started..." << std::endl;

    // 接受客户端连接
    while (*thread_exist) {
        int *new_socket = (int *)malloc(sizeof(int));
        if ((*new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0) {
            perror("accept");
            free(new_socket);
            continue;
        }

        std::cout << "Client connected!" << std::endl;

        // 创建新线程处理客户端连接
        pthread_t client_thread;
        if (pthread_create(&client_thread, nullptr, handle_client, new_socket) != 0) {
            perror("pthread_create");
            free(new_socket);
            close(*new_socket);
        }

        // 分离线程，使其自动回收
        pthread_detach(client_thread);
    }

    return nullptr;
}

void *Tcp_Service::handle_client(void *arg)
{
    int client_socket = *((int *)arg);
    free(arg);  // 释放分配的内存

    while(*thread_exist)
    {
        char buffer[1024] = {0};
        int valread = read(client_socket, buffer, 1024);
        
        if(valread <= 0) {
            // 连接关闭或出错
            if(valread == 0) {
                std::cout << "Client disconnected" << std::endl;
            } else {
                perror("read error");
            }
            break;  // 退出循环
        }

        std::cout << "Received message from client: " << buffer << std::endl;
    
        // 执行输出操作并发送回客户端
        if(strlen(buffer) > 0) {
            *ros_ctrl = true;
            std::cout << "ros_ctrl: " << *ros_ctrl << std::endl;
            if(send(client_socket, "succeed", 8, 0) < 0) {
                perror("send failed");
                break;
            }
        } else {
            if(send(client_socket, "failed", 7, 0) < 0) {
                perror("send failed");
                break;
            }
        }
    }

    // 重置控制变量
    *ros_ctrl = true;
    
    // 关闭客户端 socket
    shutdown(client_socket, SHUT_RDWR);
    close(client_socket);
    return nullptr;
}



bool Tcp_Service::start_thread()
{
    try {
        // 创建线程用于监听和接受连接
        if (pthread_create(&listen_thread, nullptr, listen_and_accept, &server_fd) != 0) {
            perror("pthread_create");
            return false;
        }

        // 分离线程，使其自动回收
        pthread_detach(listen_thread);
    } catch (const char* e) {
        // 捕获抛出的异常并处理
        std::cout << "异常捕获: " << e << std::endl;
        return false;
    }

    return true;
}
