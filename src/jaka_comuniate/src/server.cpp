#include "comuniate/server.h"
#include <iostream>
#include <string>
#include <unistd.h>
#include <ros/ros.h>
#include "comuniate/client.h"
#include "jsoncpp/json/json.h"

const int Server::max_listen;
const int Server::max_events;

Server::~Server()
{
}

int Server::setreuseaddr(int sock)
{
    int opts = SO_REUSEADDR;

    if(setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char*)&opts, sizeof(opts)) < 0){
        ROS_ERROR("sock setreuse fail.");
        return -1;
    }

    return 0;
}

int Server::setnonblocking(int sock)
{
    int opts;

    opts = fcntl(sock, F_GETFL);
    if (opts < 0) {
        std::cout<<"socket getfl failed."<<std::endl;
        return -1;
    }

    opts = opts | O_NONBLOCK;
    if (fcntl(sock,F_SETFL,opts) < 0) {
        std::cout<<"socket setfl failed."<<std::endl;
        return -1;
    }

    return 0;
}

int Server::add_read_to_epoll(int fd)
{
    struct epoll_event ev;

    memset(&ev, 0, sizeof(ev));
    ev.events = EPOLLIN | EPOLLERR;
    ev.data.fd 	= fd;
    return epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev);
}

int Server::mod_read_to_epoll(int fd)
{
    struct epoll_event ev;

    memset(&ev, 0, sizeof(ev));
    ev.events = EPOLLIN |EPOLLERR;
    ev.data.fd 	= fd;
    return epoll_ctl(epoll_fd, EPOLL_CTL_MOD, fd, &ev);
}

int Server::add_write_to_epoll(int fd)
{
    struct epoll_event ev;

    memset(&ev, 0, sizeof(ev));
    ev.events = EPOLLOUT | EPOLLERR;
    ev.data.fd 	= fd;
    return epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev);
}

int Server::mod_write_to_epoll(int fd)
{
    struct epoll_event ev;

    memset(&ev, 0, sizeof(ev));
    ev.events = EPOLLOUT |EPOLLERR;
    ev.data.fd 	= fd;
    return epoll_ctl(epoll_fd, EPOLL_CTL_MOD, fd, &ev);
}

int Server::readn(int fd, void *vptr, size_t n)
{
    return 0;
}

int Server::writen(int fd, void *vptr, size_t n)
{
    return 0;
}

static std::mutex g_tcp_mutex;

const TcpServerPtr& TcpServer::instance()
{
    static TcpServerPtr g_tcp_server;
    if(!g_tcp_server)
    {
        std::lock_guard<std::mutex> lock(g_tcp_mutex);
        if (!g_tcp_server)
        {
            g_tcp_server = std::make_shared<TcpServer>();
        }
    }

    return g_tcp_server;
}


void TcpServer::init_tcp_client()
{
    memset(users, 0, sizeof(users));
    for (int i = 0; i < MAX_USERS; ++i){
        users[i].sock = -1;
    }
}

TcpServer::TcpServer()
{
 // this->port = port;
 // this->pid = pid;

    sock_fd = socket(AF_INET,SOCK_STREAM,0);
    if (sock_fd < 0) {
        std::cout<<"create socket failed."<<std::endl;
    }
}

TcpServer::~TcpServer()
{
    if(epoll_fd > 0 && sock_fd > 0){
        epoll_ctl(epoll_fd, EPOLL_CTL_DEL, sock_fd, NULL);
    }
    close(epoll_fd);

    if(sock_fd > 0){
        close(sock_fd);
    }
}

//读,定长,在边沿触发时使用
int TcpServer::readn(int fd, void *vptr, size_t n) {
    size_t nleft;
    int nread;
    char *ptr;

    ptr = (char*)vptr;
    nleft = n;

#if 0
    /*改写以前读取方式*/
    while((nread = read(fd, ptr, nleft)) > 0){
        nleft -= nread;
        ptr += nread;
    }

    if (-1 == nread && errno != EAGAIN) {
        std::cout<<"readn error."<<std::endl;
        return -1;
    }
#endif

    int rs = 1;
    while(rs)
    {
        nread = recv(fd, ptr, nleft, 0);
        if(nread < 0)
        {
            // 由于是非阻塞的模式,所以当buflen为EAGAIN时,表示当前缓冲区已无数据可读
            // 在这里就nread当作是该次事件已处理
            if(nread == EAGAIN){
                break;
            }else{
                return -1;
            }
        }else if(nread == 0){
            // 这里表示对端的socket已正常关闭.
            std::cout<<"peer has close."<<std::endl;
            break;
        }

        if(nread != nleft){
            rs = 1;
            usleep(10000);
        }else{
            rs = 0;
        }

        nleft -= nread;
        ptr += nread;
    }

    return (n - nleft); /* return >= 0 */
}

 //写,定长
int TcpServer::writen(int fd, void *vptr, size_t n) {
    size_t nleft;
    int nwritten;
    const char *ptr;

    ptr = (char*)vptr;
    nleft = n;

#if 0
    //TODO
    while (nleft > 0) {
        nwritten = send(fd, ptr, nleft, 0);
        if (nwritten < nleft) {
            if (-1 == nwritten && errno != EAGAIN){
                std::cout<<"writen error."<<std::endl;
                return -1;
            }else if((nwritten == -1 && errno == EAGAIN) ||(0 == nwritten)){
                break;
            }
        }

        nleft -= nwritten;
        ptr += nwritten;
    }
#endif
    nwritten = send(fd, ptr, nleft, 0);
    if (nwritten < nleft) {
        if (-1 == nwritten && errno != EAGAIN){
            std::cout<<"writen error."<<std::endl;
            return -1;
        }
    }
    nleft -= nwritten;
    ptr += nwritten;

    return (n-nleft);
}

//int TcpServer::add_to_epoll(int fd, struct epoll_event& eventItem)
//{
//    return epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &eventItem);
//}

int TcpServer::setup()
{
    int rs;
    struct sockaddr_in addr_serv;
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_port = htons(port);
    addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);

    rs = 1;
    setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, (char*)&rs, sizeof(rs));

    if (bind(sock_fd, (struct sockaddr *)&addr_serv,sizeof(struct sockaddr_in)) < 0) {
        std::cout<<"socket bind error."<<std::endl;
        return -1;
    }

    if (listen(sock_fd, Server::max_listen) < 0) {
        std::cout<<"socket listen failed."<<std::endl;
        return -1;
    }

    //初始化epoll描述符
    epoll_fd = epoll_create(Server::max_events);
    if (epoll_fd <= 0) {
        std::cout<<"create epoll failed."<<std::endl;
        return -1;
    }

//    struct epoll_event ev;
//    memset(&ev, 0, sizeof(ev));
//    ev.events 	= EPOLLIN | EPOLLET; //epoll检测相应的文件的读操作将被唤醒
//    ev.events = EPOLLIN;
//    ev.data.fd 	= sock_fd;
    add_read_to_epoll(sock_fd);

   //初始化连接的client结构
    init_tcp_client();

    return 0;
}

void TcpServer::dispatchCmd(int fd, const std::string& cmd)
{
    // ResManagerPtr resPtr =  ResManager::instance();

    if(cmd.find("get #real#") != std::string::npos){
        char recv_buf[256];
        int recv_num;
        std::string strCoord;
        std::string strObj = "water";

        TcpClientPtr Testclient = std::make_shared<TcpClient>();
#if 1
        /*视觉的ip*/
        Testclient->setIp("10.55.16.76");
        Testclient->setPort(8080);

        if(!Testclient->setup()){
            std::cout<<"can not connect hikon."<<std::endl;
        }

        Testclient->sendData((char*)strObj.c_str(), strObj.length());

        for (;;) {
            if ((recv_num = recv(Testclient->getSocket(), recv_buf, sizeof(recv_buf), 0)) > 0) {
                char* tmp = (char*)recv_buf;
                tmp[recv_num] = '\0';
                std::cout<<tmp<<std::endl;
                strCoord = tmp;
                break;
            }else {
                std::cout<<"hikon client recv data error."<<std::endl;
                Testclient->closeSocket();
                // resPtr->postOrder();
                return;
            }
        }

        /**/
        Json::Reader reader;
        Json::Value root;
        if (!reader.parse(strCoord, root, false)){
            std::cout<<"coord parse error."<<std::endl;
            // resPtr->postOrder();
            return;
        }


        std::string data;
        int num = root["num"].asInt();
        if(num > 0){
            data = "{";
            for(int i=0; i<num; i++){
                char buf[10];
                sprintf(buf, "%.3f", root["point_x"][i].asDouble());
                data += buf;
                data += ",";
                sprintf(buf, "%.3f", root["point_y"][i].asDouble());
                data += buf;
            }
            data += "}";
        }
#else
        std::string data;
        data = "{";
        for(int i=0; i<6; i++){
            char buf[10];
            double f11 = 3.14156;
            sprintf(buf, "%.3f", f11);
            data += buf;
            data += ",";
            data += buf;
        }
        data += "}";

#endif
        std::cout<<data<<std::endl;
        writen(fd, (void*)data.c_str(), data.length());
        // resPtr->setFlag(true);
        // resPtr->postOrder();
    }
    // else if(cmd == "pick_finish"){
    //     resPtr->setFlag(true);
    //     resPtr->postOrder();
    // }else if(cmd == "put_finish"){
    //     resPtr->setFlag(true);
    //     resPtr->postOrder();
    // }else if(cmd == "init_finish"){
    //     resPtr->setFlag(true);
    //     resPtr->postOrder();
    // }else if(cmd == "paw_alarm"){
    //     resPtr->setFlag(false);
    //     resPtr->postOrder();
    // }

    return;
}

void TcpServer::threadFunc()
{
    int conn_fd;
    int nready;
    int recv_num;
//	int send_num;
    char recv_buf[1024];
//	char send_buf[512];
    struct sockaddr_in addr_client;
    socklen_t client_size = sizeof(struct sockaddr_in);
    struct epoll_event ev;

    while (run_flag && ros::ok()) {
        nready = epoll_wait(epoll_fd, events, max_events, 0);
        if (-1 == nready){
            std::cout<<"epoll_wait fail."<<std::endl;
        }

        for(int i = 0;i < nready; i ++) {
            //如果是sock_fd描述符可读，则创建新连接，并添加监听事件
            if (events[i].data.fd == sock_fd) {
                conn_fd = accept(sock_fd, (struct sockaddr *)&addr_client, &client_size);
                if (conn_fd < 0) {
                    std::cout<<"accept failed."<<std::endl;
                }

                std::cout<<"accept"<<std::endl;

                /* find a free user */
                int j;
                for (j = 0; j != MAX_USERS; ++j){
                    if (users[j].sock == -1){
                        break;
                    }
                }

                if (j == MAX_USERS) {
                    std::cout<<"rejected (too many users)."<<std::endl;
                    close(conn_fd);
                    break;
                }

                users[j].sock = conn_fd;
//                memset(&ev, 0, sizeof(ev));
//                ev.data.fd = conn_fd;
//                ev.events = EPOLLIN| EPOLLET;
                add_read_to_epoll(conn_fd);
            }else if(events[i].events & EPOLLIN && events[i].data.fd > 0) {
                //如果描述符可读，则读取数据后修改描述符状态去监听可写状态

                /* find a true user */
                int j;
                for (j = 0; j != MAX_USERS; ++j){
                    if (users[j].sock == events[i].data.fd){
                        break;
                    }
                }

                if (j == MAX_USERS) {
                    std::cout<<"cant find true user "<<j<<std::endl;
                    continue;
                }

                //收到数据处理,待处理,可参考框架
                if ((recv_num = recv(users[j].sock, recv_buf, sizeof(recv_buf), 0)) > 0) {
                    std::cout<<"recv num "<<recv_num<<std::endl;
                    char* str = (char*)recv_buf;
                    str[recv_num] = '\0';
                    std::cout<<str<<std::endl;

                    dispatchCmd(users[j].sock, str);
                } else if (recv_num == 0) {
                    std::cout<<"Connection closed."<<std::endl;
                    epoll_ctl(epoll_fd, EPOLL_CTL_DEL, events[i].data.fd, nullptr);
                    close(users[j].sock);

               /*   if (users[j].name != 0) {
                        _message(users[j].name, "** HAS DISCONNECTED **");
                        free(users[j].name);
                        users[j].name = 0;
                    }
                    telnet_free(users[j].telnet);
                    */
                    users[j].sock = -1;
                    events[i].data.fd = -1;
                    continue;
                } else if (errno != EINTR) {
                    std::cout<<"recv(client) failed."<<std::endl;
                    fprintf(stderr, "recv(client) failed: %s\n", strerror(errno));
                    exit(1);
                }
            }
    /*		else if(events[i].events & EPOLLOUT) {
                send_num = writen(events[i].data.fd, send_buf, strlen(send_buf));
                if (send_num < 0) {
                    epoll_ctl(epoll_fd, EPOLL_CTL_DEL, events[i].data.fd, nullptr);
                    close(events[i].data.fd);
                    events[i].data.fd = -1;
                    LOG4CXX_ERROR(logger, "write data error.");
                }

                //*假如数据没有发送完还需继续处理:TODO
                if(send_num >0){
                    LOG4CXX_ERROR(logger, "this is has left data.");
                }

                memset(&ev, 0, sizeof(ev));
                ev.data.fd = events[i].data.fd;
                ev.events = EPOLLIN | EPOLLET;
                epoll_ctl(epoll_fd,EPOLL_CTL_MOD,events[i].data.fd,&ev);
            }*/
            else{
            /*是否还有某些情况没有考虑到*/
                std::cout<<"epoll_wait failed."<<std::endl;
            }
        }

        usleep(2000);
    }
}

void TcpServer::start()
{
    thread_ = std::thread(&TcpServer::threadFunc, this);
}

void TcpServer::stop()
{
    run_flag = false;
}

static std::mutex g_udp_mutex;

const UdpServerPtr& UdpServer::instance()
{
    static UdpServerPtr g_udp_server;
    if(!g_udp_server)
    {
        std::lock_guard<std::mutex> lock(g_udp_mutex);
        if (!g_udp_server)
        {
            g_udp_server = std::make_shared<UdpServer>();
        }
    }

    return g_udp_server;
}

UdpServer::UdpServer()
{
    //初始化epoll描述符
    epoll_fd = epoll_create(Server::max_events);
    if (epoll_fd <= 0) {
        ROS_ERROR("create epoll failed.");
    }
}

UdpServer::~UdpServer()
{
    /*socket*/
    if(epoll_fd > 0 && sock_fd > 0){
        epoll_ctl(epoll_fd, EPOLL_CTL_DEL, sock_fd, NULL);
    }

    if(sock_fd > 0){
        close(sock_fd);
    }

    close(epoll_fd);
}

int UdpServer::createsock(int port)
{
    int listener;

    if ((listener = socket(PF_INET, SOCK_DGRAM, 0)) == -1){
        ROS_ERROR("socket create failed ！");
        return -1;
    }

    if(setreuseaddr(listener) < 0){
        ROS_ERROR("setreuseaddr error.");
        return -1;
    }

    if(setnonblocking(listener) < 0){
        ROS_ERROR("setnonblocking error.");
        return -1;
    }

    struct sockaddr_in addr_serv;
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_port = htons(port);
    addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(listener, (struct sockaddr *)&addr_serv,sizeof(struct sockaddr_in)) < 0) {
        ROS_ERROR("socket bind error.");
        return -1;
    }

    return listener;
}

//
int UdpServer::setup()
{
    sock_fd = createsock(port);
    if(sock_fd < 0){
        ROS_ERROR("create sock_fd fail.");
        return -1;
    }

    add_read_to_epoll(sock_fd);
    return 0;
}

void UdpServer::threadFunc()
{
    int nready;
    int recv_num;
    char recv_buf[576];
    struct sockaddr_in addr_client;
    socklen_t client_size = sizeof(struct sockaddr_in);
    run_flag  = true;

    /*循环监听*/
    while (run_flag && ros::ok()) {
        nready = epoll_wait(epoll_fd, events, Server::max_events, 0);
        if(-1 == nready){
            ROS_ERROR("epoll_wait fail.");
        }

        for(int i = 0;i < nready; i ++) {
            if (events[i].data.fd == sock_fd){
                if(events[i].events & EPOLLIN){
                    recv_num = recvfrom(sock_fd, recv_buf, 576, 0, (struct sockaddr *)&addr_client, &client_size);
                    dispatchBuf(recv_buf, recv_num);
                }else if(events[i].events & EPOLLOUT){
                    /*TODO*/
                }else if(events[i].events & EPOLLERR){
                    ROS_ERROR("head sock epoll error.");
                    break;
                }
            }else{
           //   std::cout<<"epoll_wait error socket:"<<i<<","<<events[i].data.fd<<std::endl;
            }
        }

        /*2ms period*/
        usleep(2000);
    }
}

void UdpServer::start()
{
    thread_ = std::thread(&UdpServer::threadFunc, this);
}

void UdpServer::stop()
{
    run_flag = false;
}

/*业务逻辑处理*/
void UdpServer::dispatchBuf(char* recv_buf, int len)
{

}

/***************************************************/
//unix stream socket server
/***************************************************/
static std::mutex g_stream_mutex;

const StreamServerPtr& StreamServer::instance()
{
    static StreamServerPtr g_stream_server;
    if(!g_stream_server)
    {
        std::lock_guard<std::mutex> lock(g_stream_mutex);
        if (!g_stream_server)
        {
            g_stream_server = std::make_shared<StreamServer>();
        }
    }

    return g_stream_server;
}

StreamServer::StreamServer()
{
    sock_fd = socket(PF_UNIX, SOCK_STREAM, 0);
    if (sock_fd < 0) {
        std::cout<<"create unix stream socket failed."<<std::endl;
    }
}

StreamServer::~StreamServer()
{
    if(epoll_fd > 0 && sock_fd > 0){
        epoll_ctl(epoll_fd, EPOLL_CTL_DEL, sock_fd, NULL);
    }

    close(epoll_fd);

    if(sock_fd > 0){
        close(sock_fd);
    }
}

int StreamServer::setup()
{
    unlink(UNIXSOCKETNAME);

    struct sockaddr_un addr_serv;
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sun_family = AF_UNIX;
    strcpy(addr_serv.sun_path, UNIXSOCKETNAME);

    if (bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(struct sockaddr_un)) < 0) {
        std::cout<<"socket bind error."<<std::endl;
        return -1;
    }

    if (listen(sock_fd, Server::max_listen) < 0) {
        std::cout<<"socket listen failed."<<std::endl;
        return -1;
    }

    //初始化epoll描述符
    epoll_fd = epoll_create(Server::max_events);
    if (epoll_fd <= 0) {
        std::cout<<"create epoll failed."<<std::endl;
        return -1;
    }

    add_read_to_epoll(sock_fd);

    memset(users, 0, sizeof(users));
    for (int i = 0; i < MAX_USERS; ++i){
        users[i].sock = -1;
    }

    return 0;
}

void StreamServer::start()
{
    thread_ = std::thread(&StreamServer::threadFunc, this);
}

void StreamServer::stop()
{
    run_flag = false;
}

void StreamServer::threadFunc()
{
    int conn_fd;
    int nready;
    int recv_num;
//	int send_num;
    char recv_buf[512];
    struct epoll_event ev;

    while (run_flag && ros::ok()) {
        nready = epoll_wait(epoll_fd, events, max_events, 0);
        if (-1 == nready){
            std::cout<<"epoll_wait fail."<<std::endl;
        }

        for(int i = 0;i < nready; i ++) {
            //如果是sock_fd描述符可读，则创建新连接，并添加监听事件
            if (events[i].data.fd == sock_fd) {
                conn_fd = accept(sock_fd, nullptr, nullptr);
                if (conn_fd < 0) {
                    std::cout<<"accept failed."<<std::endl;
                }

                std::cout<<"accept"<<std::endl;

                /* find a free user */
                int j;
                for (j = 0; j != MAX_USERS; ++j){
                    if (users[j].sock == -1){
                        break;
                    }
                }

                if (j == MAX_USERS) {
                    std::cout<<"rejected (too many users)."<<std::endl;
                    close(conn_fd);
                    break;
                }

                users[j].sock = conn_fd;
//              memset(&ev, 0, sizeof(ev));
//              ev.data.fd = conn_fd;
//              ev.events = EPOLLIN| EPOLLET;
                add_read_to_epoll(conn_fd);
            }else if(events[i].events & EPOLLIN && events[i].data.fd > 0) {
                //如果描述符可读，则读取数据后修改描述符状态去监听可写状态

                /* find a true user */
                int j;
                for (j = 0; j != MAX_USERS; ++j){
                    if (users[j].sock == events[i].data.fd){
                        break;
                    }
                }

                if (j == MAX_USERS) {
                    std::cout<<"cant find true user "<<j<<std::endl;
                    continue;
                }

                //收到数据处理,待处理,可参考框架
                if ((recv_num = recv(users[j].sock, recv_buf, sizeof(recv_buf), 0)) > 0) {
                    std::cout<<"recv num "<<recv_num<<std::endl;
                    char* str = (char*)recv_buf;
                    str[recv_num] = '\0';
               //   std::cout<<str<<std::endl;

               //   dispatchCmd(users[j].sock, str);
                } else if (recv_num == 0) {
                    std::cout<<"Connection closed."<<std::endl;
                    epoll_ctl(epoll_fd, EPOLL_CTL_DEL, events[i].data.fd, nullptr);
                    close(users[j].sock);

               /*   if (users[j].name != 0) {
                        _message(users[j].name, "** HAS DISCONNECTED **");
                        free(users[j].name);
                        users[j].name = 0;
                    }
                    telnet_free(users[j].telnet);
                    */
                    users[j].sock = -1;
                    events[i].data.fd = -1;
                    continue;
                } else if (errno != EINTR) {
                    std::cout<<"recv(client) failed."<<std::endl;
                    fprintf(stderr, "recv(client) failed: %s\n", strerror(errno));
                    exit(1);
                }
            }else{
            /*是否还有某些情况没有考虑到*/
                std::cout<<"epoll_wait failed."<<std::endl;
            }
        }

        usleep(2000);
    }
}

