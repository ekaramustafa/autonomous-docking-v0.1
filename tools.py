import math
import rospy
import geometry_msgs.msg
import numpy as np

from tf import transformations as t

#TESTED-APPROVED
def get_transformation_matrix(transform):
    """
    Takes an geometry_msgs.msg.Transform() object and returns its 4X4 homogenous transformation matrix

    PARAMS:
    >>>transform: geometry_msgs.msg.Transform()\n
    \n
    RETURNS:
    >>>transformation_matrix: 4x4 numpy.array
    """
    
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



#TESTED-APPROVED
def get_inverse_transform_object(transform):
    """
    Takes and transform object, find its inverse and returns inverted transform object

    PARAMS:
    >>>transform: geometry_msgs.msg.Transform()
    \n
    RETURNS:
    >>>result_transform: geometry_msgs.msg.Transform(translation,quaternion)
    """
    temp = get_transformation_matrix(transform)
    
    inv_transform_matrix = t.inverse_matrix(temp)
    
    cords = get_translation_vector(inv_transform_matrix)
    x = cords.x
    y = cords.y
    z = cords.z

    translation = geometry_msgs.msg.Vector3(x,y,z)
    
    quaternion_array = t.quaternion_from_matrix(inv_transform_matrix)
    
    result_quaternion = geometry_msgs.msg.Quaternion(quaternion_array[0],quaternion_array[1],quaternion_array[2],quaternion_array[3])

    result_transform = geometry_msgs.msg.Transform(translation,result_quaternion)
    
    return result_transform



#TESTED-APPROVED
def get_translation_vector(transformation_matrix):
    """
    Takes and 4X4 homogenous transformation matrix and returns translation vector

    PARAMS:
    >>>transformation_matrix: numpy.array, 4X4\n
    \n
    RETURNS:
    >>>translation_vector: geometry_msgs.msg.Vector3(x,y,z)
    """
    translation_array = t.translation_from_matrix(transformation_matrix)
    x = translation_array[0]
    y = translation_array[1]
    z = translation_array[2]
    translation_vector = geometry_msgs.msg.Vector3(x,y,z)
    return translation_vector



#TESTED-APPROVED
def get_rotation_matrix(transformation_matrix):
    """
    Takes an 4X4 homogenous transformation matrix and returns 3X3 rotation matrix

    PARAMS:
    >>>transformation_matrix: numpy.array, 4x4\n
    \n
    RETURNS:
    >>>rotation_matrix: numpy.array, 3x3
    """
    first_row = [transformation_matrix[0][0],transformation_matrix[0][1],transformation_matrix[0][2]]
    second_row = [transformation_matrix[1][0],transformation_matrix[1][1],transformation_matrix[1][2]]
    third_row = [transformation_matrix[2][0],transformation_matrix[2][1],transformation_matrix[2][2]]
    rotation_matrix = np.array([first_row,second_row,third_row])
    return rotation_matrix


#TESTED-APPROVED
def convert_matrix_to_transform_object(transformation_matrix):
    """
    Takes an 4X4 homogenous transformation matrix and returns and geometry_msgs.msg.Transform object

    PARAMS:
    >>>transformation_matrix: numpy.array 4x4\n
    \n
    RETURNS:
    >>>transform_object: geometry_msgs.msg.Transform(translation,quaternion)
    """
    
    #Translation: geometry_msgs.msg.Vector3
    translation = get_translation_vector(transformation_matrix)
    
    #Quaternion: geometry_msgs.msg.Quaternion
    quaternion = get_quaternion_from_transformation_matrix(transformation_matrix)

    transform_object = geometry_msgs.msg.Transform(translation,quaternion)
    
    return transform_object

#TESTED-APPROVED
def get_rotation_matrix_from_quaternion(quaternion):
    """
    Takes an geometry_msgs.msg.Quaternion(x,y,z,w) object and convert it into 3x3 rotation matrix\n
    PARAMS:
    >>>Quaternion: geometry_msgs.msg.Quaternion(x,y,z,w)\n
    \n
    RETURNS:
    >>>result: numpy.array 3x3
    """
    quat_arr = [quaternion.x,quaternion.y,quaternion.z,quaternion.w]
    result = t.quaternion_matrix(quat_arr)
    return result



#TESTED-APPROVED
def get_quaternion_from_transformation_matrix(transformation_matrix):
    """
    Takes and 4X4 homogenous transformation matrix and returns and geometry_msgs.msg.quaternion object 

    PARAMS:
    >>>transformation_matrix: numpy.array 4x4\n
    \n
    RETURNS:
    >>>quaternion: geometry_msgs.msg.Quaternion(x,y,z,w)
    """
    quaternion_array = t.quaternion_from_matrix(transformation_matrix)
    x = quaternion_array[0]
    y = quaternion_array[1]
    z = quaternion_array[2]
    w = quaternion_array[3]

    quaternion = geometry_msgs.msg.Quaternion(x,y,z,w)
    return quaternion


#TESTED-APPROVED
def get_euler_angles(quaternion):
    """
    Takes an quaternion object and returns an array of euler angles
    euler_angels = [roll angle, pitch angle, yaw angles]
    PARAMS:
    >>>quaternion: geometry_msgs.msg.Quaternion\n
    \n
    RETURNS:
    >>>res: numpy.array
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    result = t.euler_from_quaternion([x,y,z,w]) 
    return result

#TESTED-APPROVED
def multiply_transforms(transform1,transform2):
    """
    TAKES TWO geometry_msgs.msg.Transform() object, multiply them then returns

    PARAMS:
    >>>transform1,transform2: geometry_msgs.msg.Transform()\n
    \n
    RETURNS:
    >>>result: geometry_msgs.msg.Transform()
    """
    #get both 4x4 homogenous transformation matrix
    transform1_matrix = get_transformation_matrix(transform1)
    transform2_matrix = get_transformation_matrix(transform2)
    
    #multiply them
    multiplied_matrix = np.matmul(transform1_matrix,transform2_matrix)
    
    #get components of the 4x4 homogenous transformation matrix 
        #Translation: geometry_msgs.msg.Vector3(x,y,z)
    translation = get_translation_vector(multiplied_matrix)
        #rotation_quad: geometry_msgs.msg.Quaternion(x,y,z,w) 
    rotation_quad = get_quaternion_from_transformation_matrix(multiplied_matrix)
    
    result = geometry_msgs.msg.Transform(translation,rotation_quad)
    return result





