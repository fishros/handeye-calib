#include <iostream>
#include "ros/ros.h"
#include "jsoncpp/json/json.h"
#include "comuniate/client.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
using std::endl;
using namespace std;

void connect(TcpClientPtr tcpClient,string addr)
{
    tcpClient->setIp(addr);
    tcpClient->setPort(10000);
    if (tcpClient->setup())
        ROS_INFO_STREAM("connect tcp server success");
    else
        ROS_ERROR_STREAM("connect tcp server failed");
}


Json::Value recvData(TcpClientPtr tcpclient,string ip)
{
    int recv_num;
    int total_num = 0;
    char recv_buf[8192];
    std::string recvStr;

    for (;ros::ok();)
    {
        if ((recv_num = recv(tcpclient->getSocket(), &recv_buf[total_num], (sizeof(recv_buf) - total_num), 0)) > 0)
        {
            total_num += recv_num;
            if (total_num > 5000)
            {
                if (total_num >= 8192)
                    recv_buf[total_num] = '\0';
                else
                    recv_buf[sizeof(recv_buf) - 1] = '\0';
                recvStr = recv_buf;
                break;
            }
        }
        else
        {
            tcpclient->closeSocket();
            connect(tcpclient,ip);
        }
    }

    std::string::size_type begin = recvStr.find("{\"current_user_id\"");
    if (begin == recvStr.npos)
    {
        ROS_WARN("jaka client recv error data.");
    }

    std::string::size_type end = recvStr.find("{\"current_user_id\"", begin + 1);
    if (end == recvStr.npos)
    {
        ROS_WARN("jaka client recv error data.");
    }
    recvStr = recvStr.substr(begin, end);
    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(recvStr, root, false))
    {
        ROS_WARN("position parse error.");
    }
    return root;
}

double radu(double angl)
{
    return angl/180*3.1415926;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"joint_state_publisher");
    ros::NodeHandle nh;
    ros::Publisher joint_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher jaka_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("jaka_pose", 1);
    ros::Rate rate = ros::Rate(10);
    TcpClientPtr tcpClient = std::make_shared<TcpClient>();

    std::string ip;
    ros::param::get("/jaka_comuniate/jaka_host", ip);
    ROS_INFO_STREAM("Load Jaka Ip Param:"<<ip);
    if (ip == ""){
        ROS_ERROR("Can't get ip addr ,Please check param");
    }
    connect(tcpClient,ip);
    long frame_id =0;

    while (ros::ok())
    {
        Json::Value root = recvData(tcpClient,ip);
        Json::Value pos =root["joint_actual_position"];
        Json::Value acl_pos = root["actual_position"];
        if(pos.size()>0){
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(6);
            joint_state.position.resize(6);
            joint_state.name={"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
            joint_state.position = {
                radu(pos[0].asDouble()),
                radu(pos[1].asDouble()),
                radu(pos[2].asDouble()),
                radu(pos[3].asDouble()),
                radu(pos[4].asDouble()),
                radu(pos[5].asDouble())
                };
            joint_publisher.publish(joint_state);
        }

        if(acl_pos.size()>0)
        {
            geometry_msgs::PoseStamped pose;
            frame_id++;
            pose.header.frame_id = std::to_string(frame_id);
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = acl_pos[0].asDouble()/1000;
            pose.pose.position.y = acl_pos[1].asDouble()/1000;
            pose.pose.position.z = acl_pos[2].asDouble()/1000;
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(acl_pos[3].asDouble()*3.1415926/180,acl_pos[4].asDouble()/180.0*3.1415926,acl_pos[5].asDouble()*3.1415926/180);
            pose.pose.orientation.x = q.x;
            pose.pose.orientation.y = q.y;
            pose.pose.orientation.z = q.z;
            pose.pose.orientation.w = q.w;
            jaka_pose_publisher.publish(pose);
        }
      
        rate.sleep();
    }
    return 0;
}
