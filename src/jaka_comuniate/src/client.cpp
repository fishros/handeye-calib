#include "comuniate/client.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <string.h>
#include <arpa/inet.h>
#include "comuniate/common.h"

Client::~Client()
{
}

void Client::closeSocket()
{
    if(sockfd > 0){
        close(sockfd);
    }

    sockfd = 0;
    flag_close = true;
}

UdpClient::UdpClient(const std::string &ip, int port)
{
    this->ip = ip;
    this->port = port;
    sockfd = 0;
}

UdpClient::~UdpClient()
{
    if(sockfd > 0){
        close(sockfd);
    }

    sockfd = 0;
}

bool UdpClient::setup()
{
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd < 0)
    {
        printf("create udp socket error.");
        return false;
    }

    /* 设置address */
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_addr.s_addr = inet_addr(ip.c_str());
    addr_serv.sin_port = htons(port);

    flag_close = false;
    return true;
}

int UdpClient::sendData(const char *buf, int buflen)
{
    int len;
    int send_num;

    len = sizeof(addr_serv);

    send_num = sendto(sockfd, buf, buflen, 0, (struct sockaddr *)&addr_serv, len);
    if(send_num < 0)
    {
        printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
    }

}

int UdpClient::recvData(char *buf, int buflen)
{
    int len;
    int recv_num;

    len = sizeof(addr_serv);
    recv_num = recvfrom(sockfd, buf, buflen, 0, (struct sockaddr *)&addr_serv, (socklen_t *)&len);

    if(recv_num < 0)
    {
        printf("recv msg error: %s(errno: %d)\n", strerror(errno), errno);
    }

    buf[recv_num] = '\0';
}

TcpClient::TcpClient(const std::string &ip, int port)
{
    this->ip = ip;
    this->port = port;
    sockfd = 0;
}

TcpClient::~TcpClient()
{
    if(sockfd > 0){
        close(sockfd);
    }

    sockfd = 0;
}


bool TcpClient::setup()
{
    struct sockaddr_in servaddr;

    if( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        printf("create socket error: %s(errno: %d)\n", strerror(errno),errno);
        return false;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);

    //将点分十进制的ip地址转化为用于网络传输的数值格式
    if(inet_pton(AF_INET, ip.c_str(), &servaddr.sin_addr) <= 0){
        printf("inet_pton error for %s\n",ip.c_str());
        return false;
    }

    if( connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0){
        printf("connect error: %s(errno: %d)\n",strerror(errno),errno);
        return false;
    }

    flag_close = false;
    return true;
}

int TcpClient::sendData(const char *buf, int buflen)
{
    int nBytes;

    if((nBytes = send(sockfd, buf, buflen, 0)) < 0){
        printf("send buf error: %s(errno: %d)\n", strerror(errno), errno);
        return -1;
     }

    return nBytes;
}

int TcpClient::recvData(char *buf, int buflen)
{
    int nBytes;

    if((nBytes = recv(sockfd, buf, buflen, 0)) < 0){
        printf("recv buf error: %s(errno: %d)\n", strerror(errno), errno);
        return -1;
     }

     return nBytes;
}

/******************************************/
StreamClient::~StreamClient()
{
    if(sockfd > 0){
        close(sockfd);
    }

    sockfd = 0;
}

bool StreamClient::setup()
{
    struct sockaddr_un servaddr;
    memset(&servaddr,0,sizeof(servaddr));
    servaddr.sun_family=AF_UNIX;
    strcpy(servaddr.sun_path, UNIXSOCKETNAME);

    if(connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr))<0){
        printf("connect error: %s(errno: %d)\n",strerror(errno),errno);
        return false;
    }

   return true;
}


int StreamClient::sendData(const char *buf, int buflen)
{
    int nBytes;

    if((nBytes = send(sockfd, buf, buflen, 0)) < 0){
        printf("send buf error: %s(errno: %d)\n", strerror(errno), errno);
        return -1;
     }

    return nBytes;
}

int StreamClient::recvData(char *buf, int buflen)
{
    int nBytes;

    if((nBytes = recv(sockfd, buf, buflen, 0)) < 0){
        printf("recv buf error: %s(errno: %d)\n", strerror(errno), errno);
        return -1;
     }

     return nBytes;
}
