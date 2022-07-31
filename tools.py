import math
import rospy
import geometry_msgs.msg
import numpy as np

from tf import transformations as t

"""
DOES THE SAME THING WITH tf::getOrigin()
needs for explanation

PARAMS:
TRANSFORM: geometry_msgs.msg.Transform()
    - [geometry_msgs/Vector3,geometry_msgs/Quaternion]
"""
#TESTED-APPROVED
def getOrigin(transform):
    #Matrix_for_t = 4*4 homogenous transformation matrix
    matrix_for_t = get_transformation_matrix(transform)
    
    #originated is translation vector of matrix_for_t 
    originated = t.translation_from_matrix(matrix_for_t)
    #originated = [x,y,z]
    x = originated[0]
    y = originated[1]
    z = originated[2]

    return [x,y,z]


"""
RETURNS THE 4X4 HOMOGENOUS TRANSFORMATION MATRIX

PARAMS:
TRANSFORM: geometry_msgs.msg.Transform()
    - [geometry_msgs/Vector3,geometry_msgs/Quaternion]
"""
#TESTED-APPROVED
def get_transformation_matrix(transform):
    #decomposing the transform object
    
    #Quaternion: [x,y,z,w]
    quaternion = transform.rotation

    #since t.quaternion_matrix() takes an array we convert quaternion
    quaternion_arr = [quaternion.x,quaternion.y,quaternion.z,quaternion.w]

    #quaternion_matrix convert quaternion coordinates into 4x4 homogenous rotation_matrix
    rot_matrix = t.quaternion_matrix(quaternion_arr)

    #Translation: Vector3 = [x,y,z]
    translation = transform.translation

    #inits of rows of 4x4 homogenous transformation matrix
    first_row = [rot_matrix[0][0],rot_matrix[0][1],rot_matrix[0][2],translation.x]
    second_row = [rot_matrix[1][0],rot_matrix[1][1],rot_matrix[1][2],translation.y]
    third_row = [rot_matrix[2][0],rot_matrix[2][1],rot_matrix[2][2],translation.z]
    fourth_row = [0,0,0,1]
    
    #transformation_matrix : 4x4 homogenous matrix
    transformation_matrix = np.array([first_row,second_row,third_row,fourth_row])

    return transformation_matrix


"""
TAKES AN TRANSFROM OBJECT FIND ITS INVERSE AND RETURNS INVERTED TRANSFORM

PARAMS:
TRANSFORM: geometry_msgs.msg.Transform()
    - [geometry_msgs/Vector3,geometry_msgs/Quaternion]
"""
def get_inversed_transform_object(transform):
    temp = get_transformation_matrix(transform)
    
    inv_transform_matrix = t.inverse_matrix(temp)
    
    cords = get_translation_vector(inv_transform_matrix)
    x = cords[0]
    y = cords[1]
    z = cords[2]

    translation = geometry_msgs.msg.Vector3(x,y,z)

    rotation_matrix = get_rotation_matrix(inv_transform_matrix)
    
    quaternion_array = t.quaternion_from_matrix(rotation_matrix)
    
    result_quaternion = geometry_msgs.msg.Quaternion(quaternion_array[0],quaternion_array[1],quaternion_array[2],quaternion_array[3])

    result_transform = geometry_msgs.msg.Transform(translation,result_quaternion)
    
    return result_transform



"""
TAKES AN 4X4 HOMOGENOUS TRANSFORMATION MATRIX AND RETURNS TRANSLATION VECTOR

PARAMS:
TRANSFORMATION_MATRIX: numpy.array, 4X4
"""
def get_translation_vector(transformation_matrix):
    translation_array = t.translation_from_matrix(transformation_matrix)
    x = translation_array[0]
    y = translation_array[1]
    z = translation_array[2]
    translation_vector = geometry_msgs.msg.Vector3(x,y,z)
    return translation_vector


"""
TAKES AN 4X4 HOMOGENOUS TRANSFORMATION MATRIX AND RETURNS 3X3 ROTATION MATRIX

PARAMS:
TRANSFORMATION_MATRIX: numpy.array, 4X4
"""
def get_rotation_matrix(transformation_matrix):
    rotation_matrix = [
    [transformation_matrix[0][0],transformation_matrix[0][1],transformation_matrix[0][2]],
    [transformation_matrix[1][0],transformation_matrix[1][1],transformation_matrix[1][2]],
    [transformation_matrix[2][0],transformation_matrix[2][1],transformation_matrix[2][2]]]
    return rotation_matrix

"""
TAKES AN 4X4 HOMOGENOUS TRANSFORMATION MATRIX AND RETURNS AN GEOMETRY_MSGS.MSG.TRANSFORM OBJECT

PARAMS:
TRANSFORMATION_MATRIX: numpy.array 4x4
"""

def convert_matrix_to_transform_object(transformation_matrix):
    #Translation: geometry_msgs.msg.Vector3
    translation = get_translation_vector(transformation_matrix)
    
    #Quaternion: geometry_msgs.msg.Quaternion
    quaternion = get_quaternion_from_transformation_matrix(transformation_matrix)

    transform_object = geometry_msgs.msg.Transform(translation,quaternion)
    
    return transform_object

"""
TAKES TWO geometry_msgs.msg OBJECTS AND MULTIPLY THEM
"""


### DOES NOT PRODUCE CORRECT VALUES
"""
TAKES AN 4X4 HOMOGENOUS TRANSFORMATION MATRIX AND RETURNS AN GEOMETRY_MSGS.MSG.QUATERNION OBJECT 

PARAMS:
TRANSFORMATION_MATRIX: numpy.array 4x4
"""
def get_quaternion_from_transformation_matrix(transformation_matrix):

    quaternion_array = t.quaternion_from_matrix(transformation_matrix)
    x = quaternion_array[0]
    y = quaternion_array[1]
    z = quaternion_array[2]
    w = quaternion_array[3]

    quaternion = geometry_msgs.msg.Quaternion(x,y,z,w)
    return quaternion
###


###DOES NOT PRODUCE CORRECT VALUES
"""
TAKES AN QUATERNION OBJECT AND RETURNS AN ARRAY OF EULER ANGLES

PARAMS:
QUATERNION: geometry_msgs.msg.Quaternion
"""

def get_euler_angles(self,quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    quat = [x,y,z,w]

    return t.euler_from_quaternion(x,y,z,w) 
###





