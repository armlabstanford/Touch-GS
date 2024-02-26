import rospy
import numpy as np
from franka_interface import ArmInterface
from franka_interface import GripperInterface
import IPython
import quaternion
from scipy.spatial.transform import Rotation as R
from pynput import mouse, keyboard
import os
import pickle
import message_filters 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState
import cv2
import json
from json import JSONEncoder
from copy import deepcopy
from franka_core_msgs.msg import EndPointState
import glob
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_conversions
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
import random
import random

"""
:info:
    captures the dt_rgb, dt_depth, depth image, rgb_webcam image, joint info, and pose (transformation matrix) of touch, depth, and rgb camera. 

"""
# <node pkg="rosbag" type="play" name="player" args="-s 50 --clock /media/wkdo/Extreme\ SSD/vtnf_ros1bag/calibration.bag"/>



def convertxyz_np(sphere):
    # ptsnew = np.hstack((xyz, np.zeros(xyz.shape)))
    new_pts = np.zeros(sphere.shape)
    xy = sphere[:,0]**2 + sphere[:,1]**2
    new_pts[:,0] = sphere[:,0]*np.sin(sphere[:,1])*np.cos(sphere[:,2])
    new_pts[:,1] = sphere[:,0]*np.sin(sphere[:,1])*np.sin(sphere[:,2]) # for elevation angle defined from Z-axis down

    new_pts[:,2] = sphere[:,0]*np.cos(sphere[:,1])
    return new_pts

def convertSpherical_np_cam(xyz):
    """
    convert x, y, z into r, phi, theta where theta is defined in xy plane
    This ftn assumes that the y component has been flipped.
    """
    # ptsnew = np.hstack((xyz, np.zeros(xyz.shape)))
    new_pts = np.zeros(xyz.shape)
    xy = xyz[:,0]**2 + xyz[:,1]**2
    new_pts[:,0] = np.sqrt(xy + xyz[:,2]**2)
    new_pts[:,1] = np.arctan2(np.sqrt(xy), xyz[:,2]) # for elevation angle defined from Z-axis down
    #new_pts[:,1] = np.arctan2(xyz[:,2], np.sqrt(xy)) # for elevation angle defined from XY-plane up

    # to compensate cam effect 
    new_pts[:,2] = np.arctan2(-xyz[:,1], xyz[:,0])
    return new_pts

def filtering_spherical(rtp, radfiltering_upper=25.3, radfiltering_lower=0.1):
    # https://stackoverflow.com/questions/58422690/filtering-a-numpy-array
    '''
    input: (r, theta, phi)
    '''
    
    rtp_reduced = rtp[np.where((rtp[:,0]< radfiltering_upper) & (rtp[:,0]> radfiltering_lower))]
    # rtp_reduced = rtp[np.where(rtp[:,0]< radfiltering)]

    print("rad filtering - original size: ", rtp.shape, "  reduced size: ", rtp_reduced.shape)
    # print(rtp_reduced)
    return rtp_reduced


def filtering_xyz( xyz, i, zfiltering=7.5, getidx=1):
    # https://stackoverflow.com/questions/58422690/filtering-a-numpy-array
    '''
    filtering out in z direction, with given threshold
    input: (x,y,z)
    '''
        
    idx = np.where(xyz[:,2]>zfiltering)
    xyz_reduced = xyz[idx]
    i = i[idx]
    print("idx: ", getidx, " z filtering - original size: ", xyz.shape, "  reduced size: ", xyz_reduced.shape)
    # xyz_reduced[:,1] = -xyz_reduced[:,1]   

    # also change the unit into m
    return xyz_reduced/1000, i


class Record(object):
    '''
    class for initiating keyboard input
    '''

    def __init__(self):
        # rospy.init_node("path_recording")

        self.br = CvBridge()
        # self.image_tact_sub = message_filters.Subscriber('RunCamera/image_raw_1', Image)
        # self.image_webcam_sub = message_filters.Subscriber('RunCamera/webcam', Image)
        # self.image_depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        # self.image_tact_depth_sub = message_filters.Subscriber('RunCamera/imgDepth', Image)

        self.k_img_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        # self.k_depthimg_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        self.k_depthimg_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.k_touchrgb_sub = message_filters.Subscriber('RunCamera/image_raw_1', Image)
        self.k_touchdepth_sub = message_filters.Subscriber('RunCamera/imgDepth', Image)

        # ts = message_filters.ApproximateTimeSynchronizer([self.image_tact_sub, 
        #                                                 self.image_webcam_sub, 
        #                                                 self.image_depth_sub, 
        #                                                 self.image_tact_depth_sub], queue_size=10, slop=0.1, allow_headerless=True)


        ts = message_filters.ApproximateTimeSynchronizer([self.k_img_sub, 
                                                        self.k_depthimg_sub,
                                                        self.k_touchrgb_sub,
                                                        self.k_touchdepth_sub], queue_size=10, slop=0.1, allow_headerless=True)


        self.reset_flag = False
        self.record_cont_flag = 0
        self.record_all_flag = 0    
        self.count = 0          
        self.exit_flag = False

        self.savepath = os.path.join('/home/wkdo/Desktop/touchnerf/')
        self.savedepth = os.path.join(self.savepath, 'depth/train')
        self.savecolor = os.path.join(self.savepath, 'color/train')
        self.savetouch = os.path.join(self.savepath, 'touch/train')
        self.savetouch_raw = os.path.join(self.savepath, 'touch_raw/train')
        

        
        try:
            pattern = os.path.join(self.savepath, "*/train/*.jpg")
            pattern2 = os.path.join(self.savepath, "*/train/*.png")
            pattern1 = os.path.join(self.savepath, "*/train/*.npy")
            for item in glob.glob(pattern, recursive=True):
                os.remove(item)

            for item in glob.glob(pattern1, recursive=True):
                os.remove(item)
            for item in glob.glob(pattern2, recursive=True):
                os.remove(item)
            print("removed existing data")
        except FileNotFoundError:
            print("File is not present in the system.")
        
        self.result = []
        self.result_ati = []
        self.result_gr = []
        self.recorded = []


        self.path = os.path.join('/home/wkdo/Desktop/touchnerf/recorded_path/')



        self.pos = np.array([0,0,0])
        self.quat = np.quaternion(0,0,0,1)
        self.rotmat = np.eye(3)
        self.tflist_t = []
        self.tflist_tr = []
        self.tflist_r = []
        self.tflist_d = []

        self.dict_r ={
            'cameras':{
                "camera_rgb" : {
                "w": 1280, # 1920,
                "h": 720, # 1080,
                "near": 0.1,
                "far": 2.0,
                "camera_angle_x": [0.0],
                'types': ["color"]
                }
            },
            'frames' : {
                "camera": "camera_rgb",
                "file_path": "./train/r_0",
                "rotation": 0.1,
                "transform_matrix": self.rotmat.tolist(),
                "position": self.pos.tolist(),
                "quaternion": self.quat.tolist()              }   
        }     
        self.dict_d ={
            'cameras':{
                "camera_depth" : {
                "w": 1280,
                "h": 720,
                "near": 0.28,
                "far": 3.0,
                "camera_angle_x": [0.0],
                'types': ["depth"]
                }
            },
            'frames' : {
                "camera": "camera_depth",
                "file_path": "./train/r_0",
                "rotation": 0.1,
                "transform_matrix": self.rotmat.tolist(),
                "position": self.pos.tolist(),
                "quaternion": self.quat.tolist()              }   
        }     
        self.dict_tr ={
            'cameras':{
                "camera_depth" : {
                "w": 1280,
                "h": 720,
                "near": 0.28,
                "far": 3.0,
                "camera_angle_x": [0.0],
                'types': ["depth"]
                }
            },
            'frames' : {
                "camera": "camera_depth",
                "file_path": "./train/r_0",
                "rotation": 0.1,
                "transform_matrix": self.rotmat.tolist(),
                "position": self.pos.tolist(),
                "quaternion": self.quat.tolist()              }   
        }     
        self.dict_t ={
            'cameras':{
                "camera_touch" : {
                "w": 640,
                "h": 640,
                "near": 9.999999747378752e-05,
                "far": 3.27, # 15.5 -12.23, max16.88
                "camera_angle_x": [0.523598849773407],
                'types': ["touch"]
                }
            },
            'frames' : {
                "camera": "camera_touch",
                "file_path": "./train/r_0",
                "rotation": 0.1,
                "transform_matrix": self.rotmat.tolist(),
                "position": self.pos.tolist(),
                "quaternion": self.quat.tolist()     
            }   
        }     
        self.dict_tr ={
            'cameras':{
                "camera_touchraw" : {
                "w": 640,
                "h": 640,
                "near": 9.999999747378752e-05,
                "far": 3.27, # 15.5 -12.23, max16.88
                "camera_angle_x": [0.523598849773407],
                'types': ["touch_raw"]
                }
            },
            'frames' : {
                "camera": "camera_touchraw",
                "file_path": "./train/r_0",
                "rotation": 0.1,
                "transform_matrix": self.rotmat.tolist(),
                "position": self.pos.tolist(),
                "quaternion": self.quat.tolist()              }   
        }     

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        listener = keyboard.Listener(on_press=self.on_press,
                                     on_release=self.on_release)
        listener.start()

        print("---------------------description---------------------")
        print("q :       reset robot and stop (depricated)")
        print("w :       undo reset    (depricated) ")
        print("e :       start stacking the continuous trajectory")
        print("r :       save the continuous trajectory")
        print("t :       save the current data")



        ts.registerCallback(self.callback)

################# accumulated ptcloud
        self.accumulated_pcd = None  # To store accumulated point cloud
        self.pcd_publisher = rospy.Publisher("/accumulated_pcd", PointCloud2, queue_size=10)



############### for creating ptcloud and saving those from rosbag file 
                # Params
        self.image = None
        self.img_noncrop = np.zeros((768,1024))
        self.maxrad = 16.88
        self.minrad = 12.23
        self.input_width = 640
        self.imgsize = int(self.input_width/2)
        sensornum = 2 # for the sensor attached on the kinova arm, 02252024
        self.device_num = 0
        self.local_path = '/home/wkdo/catkin_ws/src/dtv2_tactile_camera/src'
        self.cen_x, self.cen_y, self.exposure = self.get_sensorinfo(sensornum)


        self.maskidx = np.load(os.path.join(self.local_path,
            'Img2Depth/calib_idx/mask_idx_{}.npy'.format(sensornum)))
        self.mask2idx = np.load(os.path.join(self.local_path,
            'Img2Depth/calib_idx/pts_2ndmask_{}_80deg.npy'.format(sensornum)))
        self.ray_vectors = np.load(os.path.join(self.local_path,
            'Img2Depth/calib_idx/pts_masked_{}.npy'.format(sensornum)))[:, 3:]


    def depth2Ptcloud(self, depthImg):

            self.img_noncrop[self.cen_y-self.imgsize:self.cen_y+self.imgsize,self.cen_x-self.imgsize:self.cen_x+self.imgsize] = depthImg
            img_vec1 = self.img_noncrop.reshape(1024*768).astype(float)
            img_vec = img_vec1/256*(self.maxrad-self.minrad)+self.minrad
            img_vec = img_vec[self.maskidx][self.mask2idx]
            # print(img_vec.shape, img_vec1[maskidx].shape, img_vec1[maskidx][mask2idx].shape )

            ray_vec = self.ray_vectors[self.mask2idx,:]
            # to ensure the same coordinate, let's use the same atan ftn from convertsperical_np ftn
            # just for angle!
            ray_vec_spherical = convertSpherical_np_cam(ray_vec)
            ray_vec_spherical[:,0] = img_vec
            # pts_masked[:,5] contains theta information
            ray_vec_spherical[:,1] = ray_vec[:,2]

            ray_vec_spherical = filtering_spherical(ray_vec_spherical, 14.5, 12.2)

            ray_xyz = convertxyz_np(ray_vec_spherical)
            

            ray_xyz_new , intensity = filtering_xyz(ray_xyz, ray_vec_spherical[:,0], 2.95)
            return ray_xyz_new
        

    def get_sensorinfo(self, calibnum):
        """
        get center of each sensor.
        """
        brightness = 160
        senarr = np.array([[6, 520, 389, 150, 320],
                            [1, 522, 343, 100, 314],
                            [2, 520, 389, 150, 322],
                            [3, 522, 343, 100, 316],
                            [4, 522, 343, 100, 307],
                            [5, 547, 384, 159, 303],
                            [101, 512, 358, 100, 298],
                            [102, 545, 379, 100, 300],
                            [103, 522, 343, 100, 300]])

        cen_x = senarr[senarr[:,0]==calibnum][0,1]
        cen_y = senarr[senarr[:,0]==calibnum][0,2]
        brightness = senarr[senarr[:,0]==calibnum][0,3]
        self.radius = senarr[senarr[:,0]==calibnum][0,4]
        return cen_x, cen_y, brightness


    def publish_accumulated_pcd(self):
        """
        Converts the accumulated numpy point cloud to PointCloud2 and publishes it.
        """
        if self.accumulated_pcd is not None:
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'touch'  # Change this to your desired frame
            # Convert numpy array to PointCloud2
            accumulated_pcd_msg = pcl2.create_cloud_xyz32(header, self.accumulated_pcd)
            self.pcd_publisher.publish(accumulated_pcd_msg)
            rospy.loginfo("Published accumulated point cloud")



    def callback(self, color, depth, touch, touch_depth):
        
        img_color = self.br.imgmsg_to_cv2(color, desired_encoding='passthrough')
        img_depth = self.br.imgmsg_to_cv2(depth, desired_encoding='passthrough')
        img_touch_raw = self.br.imgmsg_to_cv2(touch, desired_encoding='passthrough')
        img_touch = self.br.imgmsg_to_cv2(touch_depth, desired_encoding='passthrough')

        #bgr8 to rgb8
        img_color = cv2.cvtColor(img_color, cv2.COLOR_BGR2RGB)



        if self.record_all_flag == 1:


            img_depth = np.clip(img_depth, 0,3000)/3000*255
            img_depth= img_depth.astype(int)
            print(img_touch.shape)
            print(np.max(img_touch), np.min(img_touch))
            # save depth ptcloud as well

            pcd_now = self.depth2Ptcloud(img_touch)
            



            print("pcd shape: ", pcd_now.shape)

            # save the pointcloud
            if pcd_now.shape[0] > 0:
                np.save(os.path.join(self.savetouch, 'tr_{}.npy'.format(self.count)), pcd_now)
                            
                cv2.imwrite(os.path.join(self.savetouch_raw, 't_{}.png'.format(self.count)), img_touch_raw)
                cv2.imwrite(os.path.join(self.savecolor, 'c_{}.png'.format(self.count)), img_color)
                cv2.imwrite(os.path.join(self.savedepth, 'd_{}.png'.format(self.count)), img_depth)
                np.save(os.path.join(self.savedepth, 'd_{}.npy'.format(self.count)), img_depth)

                cv2.imwrite(os.path.join(self.savetouch, 'tr_{}.png'.format(self.count)), img_touch)


                # Accumulate point cloud

                # if self.accumulated_pcd is None:
                #     self.accumulated_pcd = pcd_now
                # else:
                #     self.accumulated_pcd = np.vstack((self.accumulated_pcd, pcd_now))
                
                # self.publish_accumulated_pcd()



                pos, quat, tf_rgb  = self.transform(3)
                dict_r = {
                    "camera": "camera_rgb",
                    "file_path": "./train/c_{}.png".format(self.count),
                    "rotation": 0.1,
                    "transform_matrix": tf_rgb.tolist(),
                    "position": pos,
                    "quaternion": quat               
                }  
                pos, quat, tf_depth  = self.transform(4)
                dict_d = {
                    "camera": "camera_depth",
                    "file_path": "./train/d_{}.png".format(self.count),
                    "rotation": 0.1,
                    "transform_matrix": tf_depth.tolist(),
                    "position": pos,
                    "quaternion": quat  
                }  
                pos, quat, tf_touch = self.transform(2)
                dict_tr = {
                    "camera": "camera_touchraw",
                    "file_path": "./train/t_{}.png".format(self.count),
                    "rotation": 0.1,
                    "transform_matrix": tf_touch.tolist(),
                    "position": pos,
                    "quaternion": quat  
                }  
                dict_t = {
                    "camera": "camera_touch",
                    "file_path": "./train/tr_{}.png".format(self.count),
                    "rotation": 0.1,
                    "transform_matrix": tf_touch.tolist(),
                    "position": pos,
                    "quaternion": quat  
                }  
                self.tflist_t.append(dict_t)
                self.tflist_tr.append(dict_tr)

                self.tflist_r.append(dict_r)
                self.tflist_d.append(dict_d)

                self.record_all_flag =0
                self.count +=1
                rospy.loginfo("{}-th current datapoint recorded!".format(self.count-1))

                self.dict_t['frames'] = self.tflist_t
                self.dict_tr['frames'] = self.tflist_tr
                self.dict_r['frames'] = self.tflist_r
                self.dict_d['frames'] = self.tflist_d

                json_t = json.dumps(self.dict_t, indent=4)  # use dump() to write array into file
                json_tr = json.dumps(self.dict_tr, indent=4)  # use dump() to write array into file
                json_d = json.dumps(self.dict_d, indent=4)  # use dump() to write array into file
                json_r = json.dumps(self.dict_r, indent=4)  # use dump() to write array into file

                with open(os.path.join(self.savetouch, "../transforms_train.json"), "w") as outfile:
                    outfile.write(json_t)
                with open(os.path.join(self.savetouch_raw, "../transforms_train.json"), "w") as outfile:
                    outfile.write(json_tr)
                with open(os.path.join(self.savedepth, "../transforms_train.json"), "w") as outfile:
                    outfile.write(json_d)
                with open(os.path.join(self.savecolor, "../transforms_train.json"), "w") as outfile:
                    outfile.write(json_r)


            else:
                print("not touched")
                self.record_all_flag =0


        if self.record_cont_flag == 2:
            print("saving the recorded path.....")

            self.dict_t['frames'] = self.tflist_t
            self.dict_tr['frames'] = self.tflist_tr
            self.dict_r['frames'] = self.tflist_r
            self.dict_d['frames'] = self.tflist_d

            json_t = json.dumps(self.dict_t, indent=4)  # use dump() to write array into file
            json_tr = json.dumps(self.dict_tr, indent=4)  # use dump() to write array into file
            json_d = json.dumps(self.dict_d, indent=4)  # use dump() to write array into file
            json_r = json.dumps(self.dict_r, indent=4)  # use dump() to write array into file

            with open(os.path.join(self.savetouch, "../transforms_train.json"), "w") as outfile:
                outfile.write(json_t)
            with open(os.path.join(self.savetouch_raw, "../transforms_train.json"), "w") as outfile:
                outfile.write(json_tr)
            with open(os.path.join(self.savedepth, "../transforms_train.json"), "w") as outfile:
                outfile.write(json_d)
            with open(os.path.join(self.savecolor, "../transforms_train.json"), "w") as outfile:
                outfile.write(json_r)

            print("saving done!")

            self.record_cont_flag = 0



    def get_transformation_matrix(self, frame1, frame2):


        timeout = None
        if timeout is not None:
            timeout = rospy.Duration(timeout)

        try:
            transform_stamped = self.tf_buffer.lookup_transform(frame1, frame2, rospy.Time())
            pos, quat, transform_4x4 = self.transform_stamped_to_np_array(transform_stamped)
            return pos, quat, transform_4x4

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Transform not available, please make sure that the frames are being published.")
            return None    

    def transform(self, flag=1):
        """
            listen /tf ros and convert it into transformation matrix as well as pos, and quat
            if flag = 2, transform link0 to ee_touch
            if flag = 3, transform link0 to ee_rgb
            if flag = 4, transform link0 to ee_depth
        """
        if flag == 2:
            pos, quat, T_ee = self.get_transformation_matrix("base_link", "touch")
        if flag == 3:
            pos, quat, T_ee = self.get_transformation_matrix("base_link", "camera_link")
        if flag == 4:
            pos, quat, T_ee = self.get_transformation_matrix("base_link", "camera_depth_frame")


        return pos, quat, T_ee

    def transform_stamped_to_np_array(self, transform_stamped):
        trans = transform_stamped.transform.translation
        rot = transform_stamped.transform.rotation

        quaternion = [rot.x, rot.y, rot.z, rot.w]
        translation = [trans.x, trans.y, trans.z]

        transform_matrix = tf_conversions.transformations.quaternion_matrix(quaternion)
        transform_matrix[:-1, -1] = translation

        return translation, quaternion, transform_matrix


    def on_press(self, key):
        try:
            if self.exit_flag == False:
                if key.char == 'q':
                    print('alphanumeric key {0} pressed'.format(key.char))
                    self.reset_flag = True
                if key.char == 'w':
                    print('remove reset flag')
                    self.reset_flag = False
                if key.char == 'e':
                    print('start stacking the continuous trajectory')
                    self.record_cont_flag = 1
                if key.char == 'r':
                    print('save the all information including tf info')
                    self.record_cont_flag = 2            
                if key.char == 't':
                    print('save the current data on the list ')
                    self.record_all_flag = 1            

                if key.char == 'p':
                    print("---------------------description---------------------")
                    print("q :       reset robot and stop (depricated)")
                    print("w :       undo reset    (depricated) ")
                    print("e :       start stacking the continuous trajectory")
                    print("r :       save the continuous trajectory")
                    print("t :       save the current data")
                    # print("q :       reset robot and stop ")
            if key.char=='z':
                print('end proces..')
                self.exit_flag = True
                
                return

        except AttributeError:
            print('special key {0} pressed'.format(
                key))
        if key == keyboard.Key.esc:
            # Stop listener
            rospy.loginfo("Stop keyboard listener")
            return False

    def on_release(self, key):
        # print('{0} released'.format(
        #     key))
        if key == keyboard.Key.esc:
            # Stop listener
            rospy.loginfo("Stop keyboard listener")
            return False

if __name__ == '__main__':

    # rec = Record()
    rospy.init_node("capture_data", anonymous=True)
    rec = Record()
    rospy.sleep(1.00)

    rospy.spin()    
        
    # IPython.embed()
