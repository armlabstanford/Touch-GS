#! /usr/bin/env python


"""
:info:
    03052023
    data_capturing for touchnerf using franka arm. 
    This will save the data with capture_pose.py in franka_ros_interface/test.


"""

import message_filters 
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import WrenchStamped
import glob, os
# please install netft_driver from https://github.com/armlabstanford/netft_driver
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from math import log10, sqrt
import IPython
import quaternion
from copy import deepcopy

import rospy
import IPython
from skimage.metrics import structural_similarity as compare_ssim
import argparse
from scipy.spatial.transform import Rotation as R
from franka_core_msgs.msg import RobotState, EndPointState

import csv



class listener(object):
    def __init__(self):
        self.br = CvBridge()

        self.image_sub = message_filters.Subscriber('RunCamera/image_raw_1', Image)

        self.joint_sub = message_filters.Subscriber('joint_states', JointState)

        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub,  self.joint_sub], queue_size=10, slop=0.1, allow_headerless=True)
        # ts = message_filters.TimeSynchronizer([self.image_sub, self.wrench_sub], 10)
        # ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.wrench_sub], 1,1, allow_headerless=True)



        self._cartesian_state_sub = rospy.Subscriber(
            self._ns + '/custom_franka_state_controller/tip_state',
            EndPointState,
            self._on_endpoint_state,
            queue_size=1,
            tcp_nodelay=True)

        self.savepath = os.path.join('/home/collab3/Desktop/touchnerf/recorded_dataset/')
        path = os.path.join('/home/collab3/dex_ws/src/capturedata/')
        self.rb_size = 2
        
        self.rb_img = RingBuffer(self.rb_size)
        self.rb_force = RingBuffer(self.rb_size)
        for i in range(self.rb_size):
            self.rb_img.append(cv2.imread(os.path.join(path, 'data/capture_example.jpg')))
            print(i)
        # self.image = cv2.Mat(1024,768,3)
        # self.image = cv2.imread(os.path.join(path, 'data/capture_example.png'))
        # self.image = None

        ### initialize
        for f in glob.glob(os.path.join(self.savepath, 'Img/*.jpg')):
            os.remove(f)

        with open (os.path.join(self.savepath, 'Force/force_jointinfo.csv'), 'w') as file:
            writer = csv.writer(file)
            wrlist = ['Count','Time_sec','Time_nsec',
            'Force_x','Force_y','Force_z',
            'Torque_x','Torque_y','Torque_z',
            'Jointpos_1','Jointpos_2','Jointpos_3','Jointpos_4','Jointpos_5','Jointpos_6','Jointpos_7',
            'Jointvel_1','Jointvel_2','Jointvel_3','Jointvel_4','Jointvel_5','Jointvel_6','Jointvel_7']
            writer.writerow(wrlist)
            file.close()
        self.count = -self.rb_size - 3

        ts.registerCallback(self.callback)
    def _on_endpoint_state(self, msg):

        cart_pose_trans_mat = np.asarray(msg.O_T_EE).reshape(4, 4, order='F')

        self._cartesian_pose = {
            'position': cart_pose_trans_mat[:3, 3],
            'orientation': quaternion.from_rotation_matrix(cart_pose_trans_mat[:3, :3]),
            'ori_mat': cart_pose_trans_mat[:3,:3]}

        self._cartesian_effort = {
            'force': np.asarray([msg.O_F_ext_hat_K.wrench.force.x,
                                 msg.O_F_ext_hat_K.wrench.force.y,
                                 msg.O_F_ext_hat_K.wrench.force.z]),

            'torque': np.asarray([msg.O_F_ext_hat_K.wrench.torque.x,
                                  msg.O_F_ext_hat_K.wrench.torque.y,
                                  msg.O_F_ext_hat_K.wrench.torque.z])
        }

        self._stiffness_frame_effort = {
            'force': np.asarray([msg.K_F_ext_hat_K.wrench.force.x,
                                 msg.K_F_ext_hat_K.wrench.force.y,
                                 msg.K_F_ext_hat_K.wrench.force.z]),

            'torque': np.asarray([msg.K_F_ext_hat_K.wrench.torque.x,
                                  msg.K_F_ext_hat_K.wrench.torque.y,
                                  msg.K_F_ext_hat_K.wrench.torque.z])
        }
    def endpoint_pose(self):
        """
        Return Cartesian endpoint pose {position, orientation}.

        :rtype: dict({str:np.ndarray (shape:(3,)), str:quaternion.quaternion})
        :return: position and orientation as named tuples in a dict

          - 'position': np.array of x, y, z
          - 'orientation': quaternion x,y,z,w in quaternion format

        """
        return deepcopy(self._cartesian_pose)

    def callback(self, image, wrench, joint):
        
        img_cv = self.br.imgmsg_to_cv2(image, desired_encoding='passthrough')

        self.rb_img.append(img_cv)
        self.rb_force.append(wrench)

        curr_rb = self.rb_img.get()
        self.psnr = self.getPSNR(curr_rb[self.rb_size-1], curr_rb[0])
        force_sum = np.abs(wrench.wrench.force.x) + np.abs(wrench.wrench.force.y) + np.abs(wrench.wrench.force.z) 
        if self.psnr < 0.90 and force_sum < 12.0 : 
            print("captured!", self.count)
            self.count +=1
            # remove saved buffers

            if self.count >= 0: 
            
                cv2.imwrite(os.path.join(self.savepath, 'Img/Img_{}.jpg'.format(self.count)), img_cv)


                self.save_data(wrench, joint)



    
    def save_data(self, wrench, joint):
        with open (os.path.join(self.savepath, 'Force/force_jointinfo.csv'), 'a') as file:
            writer = csv.writer(file)
            wrlist = [self.count, wrench.header.stamp.secs, wrench.header.stamp.nsecs,
                    wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z,
                    wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z]
            wrlist.extend(list(joint.position[2:]))
            wrlist.extend(list(joint.velocity[2:]))
            writer.writerow(wrlist)
    # def compare_data(self):



    def getPSNR(self, img1, img2):
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        # compute the Structural Similarity Index (SSIM) between the two
        # images, ensuring that the difference image is returned
        (score, diff) = compare_ssim(gray1, gray2, full=True)
        diff = (diff * 255).astype("uint8")
        # print("SSIM: {}".format(score))
        return score

        
    def ATI_reset(self):

        reset_ati = rospy.ServiceProxy('/ati_sensor/reset', srv.Reset)
        rospy.wait_for_service('/ati_sensor/reset', timeout = 0.5)

        reset_ati()

    def pq2tfmat(self, pos, quat):
        """
        pos and quat to transformation matrix
        """
        arquat = quaternion.as_float_array(quat)
        r = R.from_quat(arquat)
        T_obj = np.eye(4)
        T_obj[:3,:3] = r.as_matrix()
        T_obj[:3,3] = pos
        return T_obj



class RingBuffer:
    """ class that implements a not-yet-full buffer """
    def __init__(self,size_max):
        self.max = size_max
        self.data = []

    class __Full:
        """ class that implements a full buffer """
        def append(self, x):
            """ Append an element overwriting the oldest one. """
            self.data[self.cur] = x
            self.cur = (self.cur+1) % self.max
        def get(self):
            """ return list of elements in correct order """
            return self.data[self.cur:]+self.data[:self.cur]

    def append(self,x):
        """append an element at the end of the buffer"""
        self.data.append(x)
        if len(self.data) == self.max:
            self.cur = 0
            # Permanently change self's class from non-full to full
            self.__class__ = self.__Full

    def get(self):
        """ Return a list of elements from the oldest to the newest. """
        return self.data

class Test:
    def __init__(self) -> None:
        pass
    def test(self, curr):
        while True:
            for i in range(3):
                img = curr[i]        
                cv2.imshow('img', img)
                cv2.waitKey(100)

if __name__ == '__main__':



    rospy.init_node("capture_data", anonymous=True)
    listen = listener()

    rospy.spin()    

    # IPython.embed()


