#!/usr/bin/env python
# coding: utf-8
import os.path,sys,json
# env=os.path.dirname(__file__)+"sdk/"
# os.path.join(env)
# print(env)
from robotcontrol import * 

def save_file(path,data):
    if str(path).startswith("~"):
        path = path.replace("~",str(os.getenv("HOME")))
    with open(path,'a') as wf:
        wf.write(str(data))
        wf.close()
        
def main():
    # logger_init()
    Auboi5Robot.initialize()

    robot = Auboi5Robot()
    handle = robot.create_context()
    try:
        # ip = '10.55.17.38'
        ip = '10.55.130.223'
        # ip = '10.55.17.126'
        port = 8899
        result = robot.connect(ip, port)

        # tool =  { "pos": (-0.073136, -0.007195, 0.092327), "ori": (1.0, 0.0, 0.0, 0.0) }
  

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            data = ""
            while True:
                command = input("")
                if command == "r":
                    r=robot.get_current_waypoint()
                    # r = robot.base_to_base_additional_tool(r['pos'],r['ori'],tool)
                    data =  json.dumps(r) + "\n"
                    sys.stdout.write(data)
                    # save_file("./data.txt",data)
                    # break
                sys.stdout.flush()    
            robot.disconnect()
    except Exception as e:
        pass
    finally:
        if robot.connected:
            robot.disconnect()
        Auboi5Robot.uninitialize()

main()