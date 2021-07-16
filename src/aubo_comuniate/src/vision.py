# -*- coding: utf-8 -*-

from logging import logMultiprocessing
import re
import time 
import sys
import socket
import json
 
# 格式化成2016-03-20 11:45:39形式
def log(msg):
    print( "["+time.strftime("%Y-%m-%d %H:%M:", time.localtime())+"]",msg )

    
#mode ，input ：'box','bracket'
def connectScanner(mode):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 建立连接:
    conn=s.connect_ex(('127.0.0.1', 8800))
    s.settimeout(10)
    
    if conn!=0:
        log("3D视觉服务链接失败：%d" % conn)
        return {"msg_type":"failure"}
    else:
        log("3D视觉服务链接成功")
         
    # 接收欢迎消息:
    #log(s.recv(1024).decode('utf-8'))
    while True:
        if mode=='box':
           reqData={"msg_type":"location_req","obj_type":"box"}
        elif mode=='bracket':
           reqData={"msg_type":"location_req","obj_type":"bracket"}
        else:
           reqData={"msg_type":"location_req","obj_type":"box"}
           
        data2=json.dumps(reqData)
        data3=data2.encode("utf-8")
        # 发送数据:
        s.send(data3)
        #time.sleep(0.5)
        # 接收数据
        respData=s.recv(1024).decode('utf-8')
        respData2=json.loads(respData)
        log(respData2)
        #log(respData2['pose_list'][0]['x'])
        if  respData2['msg_type']=="location_rsp" and respData2['obj_type']=="box" :
            log("connectScanner recv data ok")
            return respData2
        elif respData2['msg_type']=="location_rsp" and respData2['obj_type']=="bracket" :
            log("connectScanner recv data ok")
            return respData2
        else:
            log("connectScanner recv data fail")
            return {"msg_type":"failure"}
    s.close()
    