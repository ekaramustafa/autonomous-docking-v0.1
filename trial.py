import math
import numpy 
from matplotlib.contour import QuadContourSet
import tools
import geometry_msgs.msg
from tf import transformations as t

quat = [1,0,0,1]
quat2 = [0,1,1,1]
translation = [1,2,3]
translation2 = [4,5,6]

quat1_obj = geometry_msgs.msg.Quaternion(quat[0],quat[1],quat[2],quat[3])
translation1_obj = geometry_msgs.msg.Vector3(translation[0],translation[1],translation[2])
transform_obj = geometry_msgs.msg.Transform(translation1_obj,quat1_obj)

quat2_obj = geometry_msgs.msg.Quaternion(quat2[0],quat2[1],quat2[2],quat2[3])
translation2_obj = geometry_msgs.msg.Vector3(translation2[0],translation2[1],translation2[2])
transform2_obj = geometry_msgs.msg.Transform(translation2_obj,quat2_obj)

result = tools.multiply_transforms(transform_obj,transform2_obj)

print("Transform1")
print(tools.get_transformation_matrix(transform_obj))
print()
print("Transform2")
print(tools.get_transformation_matrix(transform2_obj))
print("="*50)
print("Result")
print()
print("Translation")
print(result.translation)
print()
print("Rotation-quaternion")
print(result.rotation)
print()
print("4x4 homogenous transformation matrix")
print(tools.get_transformation_matrix(result))
