import numpy as np

def deg2rad(deg):
    return deg/180.0*np.pi

def rad2deg(rad):
    return rad/np.pi*180

def src2tgt(src, tgt):
    return np.linalg.inv(tgt).dot(src)

def rotZ(rotz):
    RotZ = np.matrix([[np.cos(rotz), -np.sin(rotz), 0, 0], 
                  [np.sin(rotz), np.cos(rotz), 0, 0], 
                  [0, 0, 1, 0], 
                  [0, 0, 0, 1]])
    return RotZ

def transZ(tz):
    TransZ = np.matrix([[1, 0, 0, 0], 
                        [0, 1, 0, 0], 
                        [0, 0, 1, tz], 
                        [0, 0, 0, 1]])
    return TransZ

def rotX(rotx):
    RotX = np.matrix([[1, 0, 0, 0], 
                      [0, np.cos(rotx), -np.sin(rotx), 0], 
                      [0, np.sin(rotx), np.cos(rotx), 0], 
                      [0, 0, 0, 1]])
    return RotX

def transX(tx):
    TransX = np.matrix([[1, 0, 0, tx], 
                        [0, 1, 0, 0], 
                        [0, 0, 1, 0], 
                        [0, 0, 0, 1]])
    return TransX

def transY(ty):
    TransY = np.matrix([[1, 0, 0, 0], 
                    [0, 1, 0, ty], 
                    [0, 0, 1, 0], 
                    [0, 0, 0, 1]])
    return TransY

def rotY(roty):
    RotY = np.matrix([[np.cos(roty), 0, np.sin(roty), 0], 
                      [0, 1, 0, 0], 
                      [-np.sin(roty), 0, np.cos(roty) , 0], 
                      [0, 0, 0, 1]])
    return RotY

class FKSolver:
    def __init__(self):
        self.angle_limits = {'s0': [-98, 98], 's1': [-123, 60],
                            'e0': [-173.5, 173.5], 'e1': [-3, 150],
                            'w0': [-175.25, 175.25], 'w1': [-90, 120],
                            'w2': [-175.25, 175.25]}
        self.direction = {'s0': 1,'s1': 1,'e0': -1,'e1': 1,'w0': -1,'w1': 1,'w2':-1}
        self.joint_keys = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
        self.DHref = {'s0':[0.064,    -0.259,  0.13,    0,    0,    -45],
                     's1':[0.069,    0,       0.27,    -90,  0,      0],
                     'e0':[0.102,    0,       0,       90,   0,     90],
                     'e1':[0.069,    0,       0.26242, -90,  -90,    0],
                     'w0':[0.10359,  0,       0,       90,   0,     90],
                     'w1':[0.01,     0,       0.2707,  -90,  -90,    0],
                     'w2':[0.115975, 0,       0,       90,   0,     90]
                     }
    def load_joints(self, joint): #expect a list
        #return {'s0':joint[0], 's1':joint[1], 'e0':joint[2], 'e1':joint[3], 'w0':joint[4], 'w1':joint[5], 'w2':joint[6]}
        return dict(zip(self.joint_keys,joint))
    def getAngle(self, rotation_m, axis, joint): 
        DH = self.DHref
        np.clip(rotation_m, -1,1, out=rotation_m)  #clamp the value
        ang = np.arccos(rotation_m[0,0])
        sin = rotation_m[0,2]
        offset = (DH[joint][4])
        if axis == 'z':
            sin = rotation_m[1,0]
            offset = DH[joint][5]
        if(np.arcsin(sin) < 0): #check if it's the lower half circle
            ang = -ang
        rot =  (rad2deg(ang)-offset)*self.direction[joint]
        return rot    
    
    def SE3(self,joint, tx,ty,tz, rotx, roty, rotz):
        rotx = deg2rad(rotx)
        roty = deg2rad(roty)
        rotz = deg2rad(rotz)
        TransX = transX(tx)
        RotX = rotX(rotx)
        TransY = transY(ty)
        RotY = rotY(roty)
        TransZ = transZ(tz)
        RotZ = rotZ(rotz)
        T = TransX.dot(TransY).dot(TransZ).dot(RotZ).dot(RotY).dot(RotX)
        return T

    def solve_poses_from_joint(self, joint_angles):
        DH = self.DHref
        direction = self.direction
        T = np.eye(4)
        intermediateT = []
        if self.check_joint_limits(joint_angles):
            for joint in self.joint_keys:
                T_i = self.SE3(joint, DH[joint][0], DH[joint][1],
                                  DH[joint][2], DH[joint][3],
                                  DH[joint][4]+direction[joint]*joint_angles[joint],DH[joint][5])
                if joint == 's0': #special case, the upper shoulder
                    T_i = self.SE3(joint, DH[joint][0], DH[joint][1],
                                  DH[joint][2], DH[joint][3],
                                  DH[joint][4],DH[joint][5]+ direction[joint]*joint_angles[joint])  
                #print('pitch', DH[joint][4]+ direction[joint]* joint_angles[joint])
                    #print T_i
                T =  T.dot(T_i)
                intermediateT.append(T)
            return intermediateT
    
    def solve_joint_from_poses(self, poses):
        DH = self.DHref
        direction = self.direction
        joint_values = [] #pitch, Roty
        #assume poses are given in array shape [4,4,7]
        for i, joint in enumerate(self.joint_keys):
            T = src2tgt(poses[:,:,i+1], poses[:,:,i]) #start with base to s0
            rot = T[:3,:3]
            rotx = deg2rad(DH[joint][3]); rotz= deg2rad(DH[joint][5]); roty= deg2rad(DH[joint][4]);
            m = (np.linalg.inv(rotZ(rotz))[:3,:3].dot(rot)).dot(np.linalg.inv(rotX(rotx))[:3,:3])
            ang = self.getAngle(m,'y',joint)
            if joint == 's0':
                m = rot.dot(np.linalg.inv(rotX(rotx))[:3,:3]).dot(np.linalg.inv(rotY(roty))[:3,:3])
                ang = self.getAngle(m,'z',joint)  
            joint_values.append(ang)
        return joint_values 
    
    def perturb_pose(self, poses, scale=20):
        joints_t = np.array(self.solve_joint_from_poses(poses))
        joints_p = joints_t + scale*np.random.randn(7) #the 
        while not self.check_joint_limits(self.load_joints(joints_p)):
            joints_p = joints_t + scale*np.random.randn(7)
        pose_p = self.solve_poses_from_joint(self.load_joints(joints_p))
        return pose_p

    def check_joint_limits(self, joint_angles):
        for joint in joint_angles.keys():
            lower_bound_check = joint_angles[joint] >= self.angle_limits[joint][0]
            upper_bound_check = joint_angles[joint] <= self.angle_limits[joint][1]
            if not (lower_bound_check and upper_bound_check):
                print "{} joint limits exceeded! angle: {}".format(joint,joint_angles[joint] )
                return False
        return True
        