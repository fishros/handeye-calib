import transforms3d as tfs
import numpy as np
import math

X,Y,Z = [ 0.0433182 ,  0.01781659 ,-0.019428  ]
RX,RY,RZ = (0.0006357429782050881, -0.0009131889099997172, 1.5589477353939896)
rmat = tfs.affines.compose(np.asarray((X,Y,Z)), tfs.euler.euler2mat(RX,RY,RZ), [1, 1, 1])


def print_mat(mat,alg=True):
    rot = tfs.euler.mat2euler(mat[0:3,0:3])
    print(str( mat[0:3, 3:4].T[0] ))
    if alg:
        print(str(math.degrees(rot[0]))+","+str(math.degrees(rot[1]))+","+str(math.degrees(rot[2])))
    else:
        print(str( rot[0] )+","+str( rot[1] )+","+str( rot[2] ))

def getValue(mat):  #
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

def opencv_pose2mat(pos,ori):  # ori: w x y z  ()  pos x y z (m)
    rot = tfs.quaternions.quat2mat((ori[0],ori[1],ori[2],ori[3]))
    rmat = tfs.affines.compose(np.asarray((pos[0],pos[1],pos[2])), rot, [1, 1, 1])
    return rmat

# real_aubo_pose  [
#     [x,y,z],   pos
#     [w,x,y,z]  ori
# ]
def getTarget(real_aubo_pose,real_camera_pose):
    real_aubo_pose = opencv_pose2mat(real_aubo_pose[0],real_aubo_pose[1])
    real_camera_pose = opencv_pose2mat(real_camera_pose[0],real_camera_pose[1])
    real_pose = np.dot(real_aubo_pose,rmat)
    real_pose = np.dot(real_pose,real_camera_pose)
    print_mat(real_pose)
    return getValue(real_pose)

def get_matrix(x,y,z,qx,qy,qz,qw):
    rmat = tfs.quaternions.quat2mat((qw,qx,qy,qz))
    rmat = tfs.affines.compose(np.squeeze(np.asarray((x,y,z))), rmat, [1, 1, 1])
    return rmat

#tool_matrix = get_matrix(-0.063136, 0.003195, 0.092327, 0.0, 0.0, 0.0, 1.0)
tool_matrix = get_matrix(-0.073136, -0.007195, 0.092327, 0.0, 0.0, 0.0, 1.0)
inverse = np.linalg.inv(tool_matrix)
# move_to_target_in_cartesian
def get_cartesian_Target(robot,pose):

    pose['ori']["rx"] = math.radians(175)
    pose['ori']["ry"] = math.radians(0)

    pose['pos']['z'] += 0.3

    ori = robot.rpy_to_quaternion( [ pose['ori']["rx"],pose['ori']["ry"],pose['ori']["rz"]  ] )
    target_mat = get_matrix( pose['pos']['x'],pose['pos']['y'],pose['pos']['z'],  ori[1],ori[2],ori[3],ori[0] )
    efftor_mat = np.dot(target_mat,inverse)

    rot = tfs.euler.mat2euler(efftor_mat[0:3,0:3])
    a,b = mat[0:3, 3:4].T[0],[math.degrees(rot[0]),math.degrees(rot[1]),math.degrees(rot[2])]
    return a,b


getTarget(
    [
       [1.2055158714112402, -0.18768288587744603, 0.3513161367841799],
       [0.0308524052650636, 0.7066564321360678, 0.7062110111519917, 0.030835429910494733]
    ],
    [
        [ 0.053845025599,0.0856517478824, 0.652839899063],
        [-0.489433306319,0.427665493117,-0.517627992416,0.556433757181]
    ]
)
