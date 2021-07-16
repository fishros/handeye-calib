#ifndef CLIENT_H
#define CLIENT_H

#include <string>
#include <memory>
#include <netinet/in.h>

class Client
{
public:
    Client(){flag_close = true;}
    virtual ~Client() = 0;
    virtual bool setup() = 0;
    virtual int sendData(const char *buf, int buflen) = 0;
    virtual int recvData(char *buf, int buflen) = 0;
    void setIp(const std::string &ip){this->ip = ip;}
    void setPort(int port){this->port = port;}
    void closeSocket();
    int getSocket(){return sockfd;}
    bool isClosed(){return flag_close;}

protected:
    std::string ip;
    int port;
    int sockfd;
    struct sockaddr_in addr_serv;
    bool flag_close;
};

using ClientPtr = std::shared_ptr<Client>;

class UdpClient:public Client{
public:
    UdpClient(){}
    UdpClient(const std::string &ip, int port);
    ~UdpClient();
    virtual bool setup();
    virtual int sendData(const char *buf, int buflen);
    virtual int recvData(char *buf, int buflen);
};

using UdpClientPtr = std::shared_ptr<UdpClient>;

class TcpClient:public Client{
public:
    TcpClient(){}
    TcpClient(const std::string &ip, int port);
    ~TcpClient();
    virtual bool setup();
    virtual int sendData(const char *buf, int buflen);
    virtual int recvData(char *buf, int buflen);
};

using TcpClientPtr = std::shared_ptr<TcpClient>;

class StreamClient:public Client{
public:
    StreamClient(){}
    ~StreamClient();
    virtual bool setup();
    virtual int sendData(const char *buf, int buflen);
    virtual int recvData(char *buf, int buflen);
};

using StreamClientPtr = std::shared_ptr<StreamClient>;

#endif // CLIENT_H
