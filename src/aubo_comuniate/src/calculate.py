import transforms3d as tfs
import numpy as np
import math

def get_matrix(x,y,z,qx,qy,qz,qw):
    rmat = tfs.quaternions.quat2mat((qw,qx,qy,qz))
    rmat = tfs.affines.compose(np.squeeze(np.asarray((x,y,z))), rmat, [1, 1, 1])
    return rmat

# handeye
X,Y,Z = [ 0.0292733  ,  0.0104433  ,  -0.0320126  ]
RX,RY,RZ = (math.radians(0.129287), math.radians(0.267614),math.radians( 90.2302))
# tool
tool_matrix = get_matrix(-0.060354, 0.000651, 0.089300, 0.0, 0.0, 0.0, 1.0)

rmat = tfs.affines.compose(np.asarray((X,Y,Z)), tfs.euler.euler2mat(RX,RY,RZ), [1, 1, 1])


def print_mat(mat,alg=True):
    rot = tfs.euler.mat2euler(mat[0:3,0:3])
    if alg:
        print(str(math.degrees(rot[0]))+","+str(math.degrees(rot[1]))+","+str(math.degrees(rot[2])))
    else:
        print(str( rot[0] )+","+str( rot[1] )+","+str( rot[2] ))

def getValue(mat):  # 米 弧度
    v = mat[0:3, 3:4].T[0]
    rot = tfs.euler.mat2euler(mat[0:3,0:3])
    return {
        "pos":{
            "x": v[0],
            "y": v[1],
            "z": v[2],
        },
        "ori":{
            "rx": rot[0],
            "ry": rot[1],
            "rz": rot[2],
        }
    }

def opencv_pose2mat(pos,ori):  # ori: w x y z  (弧度)  pos x y z (m)
    rot = tfs.quaternions.quat2mat((ori[0],ori[1],ori[2],ori[3]))
    rmat = tfs.affines.compose(np.asarray((pos[0],pos[1],pos[2])), rot, [1, 1, 1])
    return rmat


def getTarget(real_aubo_pose,real_camera_pose): # 工具坐标
    real_aubo_pose = opencv_pose2mat(real_aubo_pose[0],real_aubo_pose[1])
    real_camera_pose = opencv_pose2mat(real_camera_pose[0],real_camera_pose[1])
    real_pose = np.dot(real_aubo_pose,rmat)
    real_pose = np.dot(real_pose,real_camera_pose)
    return getValue(real_pose)




#tool_matrix = get_matrix(-0.073136, -0.007195, 0.092327, 0.0, 0.0, 0.0, 1.0)
inverse = np.linalg.inv(tool_matrix)

# move_to_target_in_cartesian
# 返回
# pos:位置坐标（x，y，z），单位(m)
# rpy：欧拉角（rx，ry，rz）,单位（度）
def get_cartesian_Target(robot,pose):
    pose['ori']["rx"] = math.radians( 179.98 )
    pose['ori']["ry"] = math.radians( 0 )
    
    # rz 有可能超过 +- 175  +180结果
    _ori =  [ pose['ori']["rx"], pose['ori']["ry"], pose['ori']["rz"]   ] 

    ori = robot.rpy_to_quaternion(_ori)
    target_mat = get_matrix( pose['pos']['x'],pose['pos']['y'],pose['pos']['z'],  ori[1],ori[2],ori[3],ori[0] )
    efftor_mat = np.dot(target_mat,inverse)
    
    # 旋转90
    # efftor_mat = np.dot(efftor_mat,get_matrix(0, 0, 0, 0.0,  0.0, 0.0, math.radians( 90 )))

    rot = tfs.euler.mat2euler(efftor_mat[0:3,0:3])
    a,b = efftor_mat[0:3, 3:4].T[0],[math.degrees(rot[0]),math.degrees(rot[1]),math.degrees(rot[2])]

    print("raw",a,b)

    b[0] = 179.98
    b[1] = 0
    a[2] += 0.005
    return a,b
