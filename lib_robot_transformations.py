import math
import numpy as np
#import pandas as pd
from numpy import linalg
import copy
import warnings

# KUKA  = XYZ ABC
# FANUC = XYZ WPR

# around axis   X      Y      Z
#              ROLL  PITCH   YAW
# KUKA          C      B      A
# FANUC         W      P      R


# KUKA Rotation_matrix = Rz(A) * R'y(B) * R''x(C)
# FANUC Rotation_matrix = (Rx(-W) * Ry(-P) * Rz(-R)).T


class Quaternion:
    def __init__(self,parent_type, w = 1, x = 0, y = 0, z = 0 ):
        self.w = w
        self.x = x
        self.y = y
        self.z = z
        
        self.parent_type = parent_type
        
    def __repr__(self):
        return str(type(self)) + "\n" + 'parent_type: ' + str(self.parent_type) + "\n" +  "{" + " w: " + str(self.w) + " x: " + str(self.x) + " y: " + str(self.y) + " z: " + str(self.z) + "}"   


    def __mul__(self, another):
        if isinstance(another, Quaternion):
#self         another
        # quaternion1, quaternion0
            new_w = -self.x * another.x - self.y * another.y - self.z * another.z + self.w * another.w
            new_x = self.x * another.w + self.y * another.z - self.z * another.y + self.w * another.x
            new_y = -self.x * another.z + self.y * another.w + self.z * another.x + self.w * another.y
            new_z = self.x * another.y - self.y * another.x + self.z * another.w + self.w * another.z
            
            new_parent_type = self.parent_type
            return Quaternion(new_parent_type , new_w, new_x, new_y, new_z )
                                                 
        elif isinstance(another, int):
            return Quaternion(self.parent_type, self.w * another, self.x * another, self.y * another, self.z * another)

        elif isinstance(another, float):
            return Quaternion(self.parent_type, self.w * another, self.x * another, self.y * another, self.z * another)

        else:
            return None

    def __rmul__(self, another):
        if isinstance(another, int):
            return Quaternion(self.parent_type, self.w * another, self.x * another, self.y * another, self.z * another)

        elif isinstance(another, float):
            return Quaternion(self.parent_type, self.w * another, self.x * another, self.y * another, self.z * another)
        
    


    def numpy(self):
        return np.array([self.w, self.x, self.y, self.z]).astype(np.float32)

    def get_angles(self):
  
        # A yaw around Z
        yaw = np.degrees(np.arctan2(2 * (self.w * self.z + self.x * self.y) ,
                     1.0 - 2.0 * (self.y **2 + self.z**2))) #* 180 / np.pi
        # B pitch around Y

        t2 = 2.0 * (self.w * self.y - self.z* self.x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.degrees(np.arcsin(t2))
    #     B = math.degrees(np.arctan2(1+ 2 * (quaternion['w'] * quaternion['y'] - quaternion['x'] * quaternion['z']),
    #                                (1 - 2* (quaternion['w'] * quaternion['y'] - quaternion['x'] * quaternion['z']))))# * 180 / np.pi
        
        # C roll around X
        roll = np.degrees(np.arctan2(2 * (self.w * self.x + self.y * self.z) , (1.0 - 2.0* (self.x**2 + self.y**2)))) # * 180 / np.pi
        
        # print('yaw: ', yaw, 'pitch: ', pitch, 'roll: ', roll)
        return np.array([yaw, pitch, roll]).astype(np.float32)


    def get_norm(self):
        return np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)


    def inv(self):
        # q^-1 = (w, -x, -y, -z) / (w^2 + x^2 + y^2 + z^2)
        res = np.array([self.w, - self.x, - self.y, -self.z]).astype(np.float32) / (self.w**2 + self.x**2 + self.y**2 + self.x**2)
        return Quaternion(self.parent_type, res[0], res[1], res[2], res[3])


    def conj(self):
        return Quaternion(self.parent_type , self.w, - self.x, - self.y, - self.z )

        
   
        
        
        
        
        




#################################################################################################
#################################################################################################
class Rotation_matrix:
    def __init__(self, mtx, parent_type):       
        
        self.mtx = mtx
        self.parent_type = parent_type            
    
    def extract_frame(self):
        if self.parent_type == 'KUKA':
            yaw, pitch, roll = angles_from_rot_mtx(self.mtx) 
            frame = KUKA_frame(self.mtx[0,3], self.mtx[1,3], self.mtx[2,3], yaw, pitch, roll)
        
        elif self.parent_type == 'FANUC':
            roll, pitch, yaw = angles_from_rot_mtx(self.mtx)            
            frame = FANUC_frame(self.mtx[0,3], self.mtx[1,3], self.mtx[2,3], yaw, pitch, roll)
            
        else:
            raise Exception('Can not extract frame from matrix. Unknown type of frame')            
            
        return frame    
            
        
    def inv(self):
        inv_mtx_obj = Rotation_matrix(np.linalg.inv(self.mtx), parent_type = self.parent_type)
        
        return inv_mtx_obj
            
            
    def description(self):
        print('-------------------------')
        print(self.mtx)
        print('parent_type: ', self.parent_type)
        print('-------------------------')
    
    def __repr__(self):
        return str(type(self)) + "\n" + 'parent_type: ' + str(self.parent_type) + "\n" +  str(self.mtx)
    
    def __mul__(self, another):
        if self.parent_type != another.parent_type:
            print('Warning! Different parent types of matrices. "{}" type has been assigned to result matrix'.format(self.parent_type))

            
        new_mtx = np.dot(self.mtx, another.mtx)
        # print('new_mtx: ', new_mtx)
        return Rotation_matrix(new_mtx, parent_type = self.parent_type)
        
              
        
        
#################################################################################################
#################################################################################################
class KUKA_frame: # XYZ ABC
    def __init__(self, x = 0, y = 0, z = 0, a = 0, b = 0, c = 0):
        self.x = x
        self.y = y
        self.z = z
        self.a = a # Rz
        self.b = b # Ry
        self.c = c # Rx
    
    def __str__(self):
        string = '(X = ' + str(self.x) + ', Y = ' + str(self.y) + ', Z = ' + str(self.z) + ', A = ' + str(self.a) + ', B = ' + str(self.b) + ', C = ' + str(self.c) + ')'
        return string
    
    def __repr__(self):
        return str(type(self)) + "\n" +  str({"x":self.x, "y":self.y, "z":self.z, "a":self.a, "b":self.b, "c":self.c})
    
    def __mul__(self, another):
#         if type(another) != KUKA_frame:
#             raise Exception('different types of frames')
        
#         else:
        return (self.rot_mtx() * another.rot_mtx()).extract_frame()
            
        
    def inv(self):
        return self.rot_mtx().inv().extract_frame()
 
    
    def to_tuple(self):
        return (round(self.x,6), round(self.y, 6), round(self.z, 6), round(self.a, 6), round(self.b, 6), round(self.c, 6))

    def to_list(self):
        return [round(self.x,6), round(self.y, 6), round(self.z, 6), round(self.a, 6), round(self.b, 6), round(self.c, 6)]
    
    
    def rot_mtx(self):
        
        mtx = transform_mtx_from_frame(self)
        return Rotation_matrix(mtx = mtx, parent_type = 'KUKA' )
    
    def round_elements(self, num : int):
        return KUKA_frame(
            round(self.x, num),
            round(self.y, num),
            round(self.z, num),
            round(self.a, num),
            round(self.b, num),
            round(self.c, num)
            )
            
       
    

#################################################################################################
#################################################################################################    
class FANUC_frame: # XYZ WPR
    def __init__(self, x = 0, y = 0, z = 0, w = 0, p = 0, r = 0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w 
        self.p = p
        self.r = r
    
    def __str__(self):
        string = '(X = ' + str(self.x) + ', Y = ' + str(self.y) + ', Z = ' + str(self.z) + ', W = ' + str(self.w) + ', P = ' + str(self.p) + ', R = ' + str(self.r) + ')'
        return string
    
    def __repr__(self):
        return str(type(self)) + "\n" +  str({"x":self.x, "y":self.y, "z":self.z, "w":self.w, "p":self.p, "r":self.r})
    
    def __mul__(self, another):
#         if type(another) != FANUC_frame:
#             raise Exception('different types of frames')
        
#         else:
        return (self.rot_mtx() * another.rot_mtx()).extract_frame()
        
        
    def inv(self):
        return self.rot_mtx().inv().extract_frame()        
        
        
    def to_tuple(self):
        return (round(self.x,6), round(self.y, 6), round(self.z, 6), round(self.w, 6), round(self.p, 6), round(self.r, 6))    
    
    def rot_mtx(self):
        mtx = transform_mtx_from_frame_extrinsic(self)
        return Rotation_matrix(mtx = mtx, parent_type = 'FANUC' )

    
    
    
#################################################################################################
#################################################################################################    
def angles_from_rot_mtx(rot_mtx):
    A = np.arctan2(rot_mtx[1,0], rot_mtx[0,0])
    sin_A = math.sin(A)
    cos_A = math.cos(A)
    
    sin_B = -1 * rot_mtx[2,0]
    abs_cos_B = cos_A * rot_mtx[0,0] + sin_A * rot_mtx[1,0]
    B = np.arctan2(sin_B, abs_cos_B)
    
    sin_C = sin_A * rot_mtx[0,2] - cos_A * rot_mtx[1,2]
    cos_C = - 1 * sin_A * rot_mtx[0,1] + cos_A * rot_mtx[1,1]
    C = np.arctan2(sin_C, cos_C)
    
    return np.degrees(A), np.degrees(B), np.degrees(C)

def extract_frame(transform_matrix):
    a, b, c = angles_from_rot_mtx(transform_matrix)
   
    frame = KUKA_frame(transform_matrix[0,3], transform_matrix[1,3], transform_matrix[2,3], a, b, c)
      
    return frame


def rot_mtx_around_Z(a): # A - angle around Z axis in degrees 
    Angle_around_Z = math.radians(a)  # Angle A
    
    matrix_turn_around_Z = np.array([
        [math.cos(Angle_around_Z), -1 * math.sin(Angle_around_Z), 0, 0],
        [math.sin(Angle_around_Z), math.cos(Angle_around_Z), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    return matrix_turn_around_Z

def rot_mtx_around_Y(b): # B - angle around Y axis in degrees 
    Angle_around_Y = math.radians(b)  # Angle B
    
    matrix_turn_around_Y = np.array([
        [math.cos(Angle_around_Y), 0, math.sin(Angle_around_Y), 0],
        [0, 1, 0, 0],
        [-1 * math.sin(Angle_around_Y), 0,  math.cos(Angle_around_Y), 0],
        [0, 0, 0, 1]
    ])

    return matrix_turn_around_Y

def rot_mtx_around_X(c): # C - angle around Y axis in degrees 
    Angle_around_X = math.radians(c)  # Angle C

    matrix_turn_around_X = np.array([
        [1, 0, 0, 0],
        [0, math.cos(Angle_around_X), -1 * math.sin(Angle_around_X), 0],
        [0, math.sin(Angle_around_X), math.cos(Angle_around_X), 0],
        [0, 0, 0, 1]
    ])
    
    return matrix_turn_around_X

def rot_mtx_vector(x, y, z):
    mtx = np.eye(4)
    mtx[0,3] = x
    mtx[1,3] = y
    mtx[2,3] = z
    mtx[3,3] = 1
    return mtx
    
# INTRINSIC mtx = Rz * R'y * R''x
def transform_mtx_from_frame(frame): 
    matrix_turn_around_Z = rot_mtx_around_Z(frame.a)
    matrix_turn_around_Y = rot_mtx_around_Y(frame.b)
    matrix_turn_around_X = rot_mtx_around_X(frame.c) 
    vector_mtx = rot_mtx_vector(frame.x, frame.y, frame.z)
    
    evolution_matrix = np.dot( np.dot( np.dot( vector_mtx, matrix_turn_around_Z), matrix_turn_around_Y), matrix_turn_around_X)
    
    return evolution_matrix

 
def inv_pos(frame): #FRAME as input    
    evolution_matrix = transform_mtx_from_frame(frame) #turn matrix
        
    #make inverse matrix
    inv_matrix = np.linalg.inv(evolution_matrix)
    
    #Turt matrix to frame
    
    inv_frame = extract_frame(inv_matrix)
    #print(inv_frame)
    
    return inv_frame


# Reorient tool in reference to TOOL frame axis
def rotateInToolCoord(source_point, shift_frame_Angles): # x,y,z must be zero
    #RESULT = source_point : shift_frame_Angles (and stay the initial point unchanged)
    
    shift_frame_Angles.x=0
    shift_frame_Angles.y=0
    shift_frame_Angles.z=0
    result_frame = extract_frame( np.dot(transform_mtx_from_frame(source_point), transform_mtx_from_frame(shift_frame_Angles)))
    return result_frame


# Reorient tool in reference to BASE frame axis
def rotateInBaseCoord(source_point : KUKA_frame, shift_frame_Angles : KUKA_frame) -> KUKA_frame:     # x,y,z must be zero
    #RESULT = shift_frame_Angles : source_point (and stay the initial point unchanged)
    
    shift_frame_Angles.x=0
    shift_frame_Angles.y=0
    shift_frame_Angles.z=0


    tempFrame = copy.copy(source_point)
    tempFrame.x = 0
    tempFrame.y = 0
    tempFrame.z = 0
    
    # print('TempFrame', TempFrame)
    
    tempFrame = extract_frame(np.dot(transform_mtx_from_frame(shift_frame_Angles), transform_mtx_from_frame(tempFrame) ))
    # print('TempFrame', TempFrame)
    
    result_frame =  source_point
    
    # print('Result_frame', Result_frame)
    
    result_frame.a =  tempFrame.a
    result_frame.b =  tempFrame.b
    result_frame.c =  tempFrame.c
    
    return result_frame

def from_base_to_base(source_point : KUKA_frame, source_base : KUKA_frame, target_base : KUKA_frame) -> KUKA_frame:
    # Source point doesn't move. Just recalculated in reference to Target_base
        
    temp_mtx = np.dot(transform_mtx_from_frame(source_base), transform_mtx_from_frame(source_point))
    temp_mtx = np.dot(transform_mtx_from_frame(inv_pos(target_base)), temp_mtx)

    result_frame = extract_frame(temp_mtx)
    
    return result_frame

# EXTRINSIC for FANUC  XYZ WPR.   angles(W = Rx, P = Ry, R = Rz)
def transform_mtx_from_frame_extrinsic(frame): 
    matrix_turn_around_Z = rot_mtx_around_Z(-frame.r)
    matrix_turn_around_Y = rot_mtx_around_Y(-frame.p)
    matrix_turn_around_X = rot_mtx_around_X(-frame.w) 
    vector_mtx = rot_mtx_vector(frame.x, frame.y, frame.z)
    
    
    evolution_matrix = np.dot(vector_mtx, np.dot( np.dot(matrix_turn_around_X, matrix_turn_around_Y), matrix_turn_around_Z).T)
    return evolution_matrix


# KUKA  = XYZ ABC
# FANUC = XYZ WPR

# around axis   X      Y      Z
#              ROLL  PITCH   YAW
# KUKA          C      B      A
# FANUC         W      P      R


def angles_to_quaternion(np_arr_yaw_pitch_roll):  
    
    
    yaw = np.radians(np_arr_yaw_pitch_roll[0])
    pitch = np.radians(np_arr_yaw_pitch_roll[1])
    roll = np.radians(np_arr_yaw_pitch_roll[2])
    
    
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    
    return Quaternion('KUKA', qw, qx, qy, qz)


def quaternion_to_angles(quaternion):
    # A yaw around Z
    yaw = math.degrees(np.arctan2(2 * (quaternion['w'] * quaternion['z'] + quaternion['x'] * quaternion['y']) ,
                 1 - 2 * (quaternion['y']**2 + quaternion['z']**2))) #* 180 / np.pi
    # B pitch around Y
    pitch = math.degrees(np.arcsin(2 * (quaternion['w'] * quaternion['y'] - quaternion['z'] * quaternion['x'])))# * 180 / np.pi
#     B = math.degrees(np.arctan2(1+ 2 * (quaternion['w'] * quaternion['y'] - quaternion['x'] * quaternion['z']),
#                                (1 - 2* (quaternion['w'] * quaternion['y'] - quaternion['x'] * quaternion['z']))))# * 180 / np.pi
    
    # C roll around X
    roll = math.degrees(np.arctan2(2 * (quaternion['w'] * quaternion['x'] + quaternion['y'] * quaternion['z']) ,
                 1 - 2 * (quaternion['x']**2 + quaternion['y']**2)))# * 180 / np.pi
    # print('yaw: ', yaw, 'pitch: ', pitch, 'roll: ', roll)
    return roll, pitch, yaw
