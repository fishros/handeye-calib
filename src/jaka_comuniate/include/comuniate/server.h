#ifndef SERVER_H
#define SERVER_H

#include <sys/types.h>
#include <netinet/in.h>
#include <sys/epoll.h>
#include <sys/un.h>
#include <fcntl.h>
#include <string.h>
#include <string>
#include "common.h"

#define MAX_USERS 64
#define SOCKET int

struct user_t {
//  char *name;
    SOCKET sock;
};

#pragma pack(1)
typedef struct _tagPixel{
  char ch1;
  float pixel1;
  char ch2;
  float pixel2;
  char ch3;
  float pixel3;
  char ch4;
  float pixel4;
  char ch5;
  float pixel5;
  char ch6;
  float pixel6;
  char ch7;
}Pixel;
#pragma pack()

class Server{
public:
    virtual ~Server() = 0;
    virtual int setup() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;

    int setnonblocking(int sock);
    int setreuseaddr(int sock);
    void setPort(int port){this->port = port;}
    int add_read_to_epoll(int fd); /*读*/
    int mod_read_to_epoll(int fd); /*读*/

    int add_write_to_epoll(int fd); /*写*/
    int mod_write_to_epoll(int fd); /*写*/
    virtual int readn(int fd, void *vptr, size_t n);
    virtual int writen(int fd, void *vptr, size_t n);

    void join(){thread_.join();}
    int get_sock(){return sock_fd;}

public:
    static const int max_listen = 5;
    static const int max_events = 16;
    struct epoll_event events[max_events];

protected:
    int port;
    int sock_fd;
    int epoll_fd;
    std::thread thread_;
    std::atomic_bool run_flag;
};

typedef std::shared_ptr<Server> ServerPtr;

class TcpServer;
typedef std::shared_ptr<TcpServer> TcpServerPtr;

class TcpServer:public Server{
public:
    static const TcpServerPtr& instance();
    TcpServer();
    ~TcpServer();
    virtual int setup();
    virtual void start();
    virtual void stop();
    int readn(int fd, void *vptr, size_t n);
    int writen(int fd, void *vptr, size_t n);
    void threadFunc();

public:
    TcpServer & operator =( const TcpServer & ) = delete;
    TcpServer( const TcpServer& ) = delete;

private:
    struct user_t users[MAX_USERS];
    void init_tcp_client();
    void dispatchCmd(int fd, const std::string& cmd);
};

class UdpServer;
typedef std::shared_ptr<UdpServer> UdpServerPtr;

class UdpServer:public Server{
public:
    static const UdpServerPtr& instance();
    UdpServer();
    ~UdpServer();
    virtual int setup();
    virtual void start();
    virtual void stop();
    void threadFunc();

public:
    UdpServer & operator =( const UdpServer & ) = delete;
    UdpServer( const UdpServer& ) = delete;

private:
    void dispatchBuf(char* recv_buf, int len);
    int createsock(int port);
};

class  StreamServer;
typedef std::shared_ptr<StreamServer> StreamServerPtr;

class  StreamServer:public Server{
public:
    static const StreamServerPtr& instance();
    StreamServer();
    ~StreamServer();
    virtual int setup();
    virtual void start();
    virtual void stop();
    void threadFunc();

public:
    StreamServer & operator =( const StreamServer & ) = delete;
    StreamServer( const StreamServer& ) = delete;

private:
    struct user_t users[MAX_USERS];
};

#endif
