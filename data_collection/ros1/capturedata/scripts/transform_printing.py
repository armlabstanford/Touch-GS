import numpy as np

from scipy.spatial.transform import Rotation as R

"""
:info:
    captures the dt_rgb, dt_depth, depth image, rgb_webcam image, joint info, and pose (transformation matrix) of touch, depth, and rgb camera. 

"""

"""
convert position of ef pose frame to the pose of the gel
or convert position of real ee pose to the fake ee pose for the cartesian command 
##########
flange quat is bullshit, let's use ori_mat in endpoint_pose()
rotation2quat or quat2rotation are also messing the rotation, let's not use them
###########

:param pos / quat: fake end-effector pose or desired end effector pose 
:type pos: [float] or np.ndarray
:type quat: quaternion.quaternion or [float] (quaternion in w,x,y,z order)
:type rotmat: np.ndarray((3,3)) rotation matrix

ef: fake end effector, 
ee_touch: pose of DenseTact (touch)
ee_rgb: pose of rgb camera (rgb)

:param flag: if flag = 1, transform ee to ef (for giving commands)
            if flag = 2, transform ef to ee_touch
            if flag = 3, transform ef to ee_rgb
            if flag = 4, transform ef to ee_depth
            
return: pos, quat, tfmat of the true end effector 
"""

flag = 3
T_now = np.eye(4)

ee_rot2 = 0.707099974155426
ee_z = 0.10339999943971634

# flange (link 8 ) is located in 4.5mm inside of the bolted part 
fl_distance =0.004212608
touch_distance = (48.57+1.7)/1000
lenspos = 2 # 2mm from outer glass
rgb_distance_z = (46 - lenspos)/1000
rgb_distance_x = 96.85/1000
depth_distance_z = 30.85/1000
depth_distance_x = 50/1000
depth_distance_y = -17.5/1000

touch_finz = fl_distance + touch_distance -ee_z
depth_finz = depth_distance_z + fl_distance - ee_z
rgb_finz = rgb_distance_z + fl_distance - ee_z


# ef to ee
T_ef2ee = np.eye(4)
T_ee = np.eye(4)

if flag == 1:
    # convert from ee to ef for checking (don't use this for autonomous case - trajectory needed)
    T_w2ee = T_now
    # for flag 1, use the edge of the densetact 1 (add 25.5mm)
    T_ef2ee[:3,3] = np.array([0,0,touch_finz+25.5/1000])
    T_ee = np.dot(T_w2ee, np.linalg.inv(T_ef2ee))

if flag == 2:
    # ef to ee_touch 
    T_w2ef = T_now
    T_ef2ee[:3,3] = np.array([0,0,touch_finz])
    T_ee = np.dot(T_w2ef, T_ef2ee)

if flag == 3:
    # ef to ee_rgb 
    T_w2ef = T_now
    T_ef2ee[:3,3] = np.array([rgb_distance_x,0,rgb_finz])
    T_ee = np.dot(T_w2ef, T_ef2ee)

if flag == 4:
    # ef to ee_depth 
    T_w2ef = T_now
    T_ef2ee[:3,3] = np.array([depth_distance_x, depth_distance_y, depth_finz])
    T_ee = np.dot(T_w2ef, T_ef2ee)

print(T_ee)

#  IPython.embed()

