import rospy
import math
import avg
import rospy
import geometry_msgs.msg
import numpy as np

from tf import transformations as t

class paper_tools():
    def __init__(self) -> None:
        pass

    """
    DOES THE SAME THING WITH tf::getOrigin()
    """
    def getOrigin(self,transform):
        #Matrix_for_t = 4*4 homogenous transformation matrix
        matrix_for_t = self.get_transformation_matrix(transform)
        
        #originated is translation vector of matrix_for_t 
        originated = t.translation_from_matrix(matrix_for_t)
        #originated = [x,y,z]
        x = originated[0]
        y = originated[1]
        z = originated[2]

        return [x,y,z]


    """
    TRANSFORM: geometry_msgs.msg.Transform()
        - [geometry_msgs/Vector3,geometry_msgs/Quaternion]
    """
    def get_transformation_matrix(self,transform):
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
    
    
