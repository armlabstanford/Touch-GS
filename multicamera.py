# A simple script that uses blender to render views of a single object by rotation the camera around it.
# Also produces depth map at the same time.

import argparse, sys, os
import json
import bpy
from mathutils import Vector
import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import gc
import random
import pprint
import bmesh
import sys

#camera = 'Camera'  #'Camera.001' # 'Camera'
#VIEWS = 400
##NUM = 800
#RESOLUTION_X = 600#1024 #800
#RESOLUTION_Y = 480#1024 #600
fib_radius = 0.85
RESULTS_PATH = 'data'
CAMERA_TYPE = ['color', 'depth', 'touch']
TRAIN_TYPE = ['train']#, 'test']#, 'val']
FORMAT = 'PNG'
specified_offset_units = 0.0
offset_distance = 0.01
#radius = .115 # remember the box is centered on origin and is with halflength of 1m
#zcutoff = None
fp = bpy.path.abspath(f"//{RESULTS_PATH}")
bpy.context.scene.render.use_lock_interface = True


# Lambda rotation Euler rotation matrices, x, y, and z
Rx = lambda theta: np.array([[1, 0, 0],
                             [0, np.cos(theta), -1*np.sin(theta)],
                             [0, np.sin(theta), np.cos(theta)]
                            ])
Ry = lambda phi: np.array([[np.cos(phi), 0, np.sin(phi)],
                           [0,1,0],
                           [-1*np.sin(phi), 0, np.cos(phi)]
                          ])
Rz = lambda psi: np.array([[np.cos(psi), -1*np.sin(psi),0],
                           [np.sin(psi),np.cos(psi),0],
                           [0,0,1]
                          ])
              
              
# generate camera origin viewpoints:
def gen_viewpoints(camera):
    # print("radius used:")
    # print(camera["radius"])
    if camera["pose_method"] == "fibonacci_sphere":
        points = fibonacci_sphere(r=camera["radius"], zcutoff=camera["zcutoff"], samples=camera["num_views"])
        return points
        
    elif camera["pose_method"] == "cube_look":
        points = cube_look(r=camera["radius"], samples=camera["num_views"])
        return points
        
    elif camera["pose_method"] == "cone_look":
        points = cone_look(radius=camera["circle_r"],height=camera["radius"],
                           phi=camera["phi"], theta=camera["theta"],
                           psi=camera["psi"], samples=camera["num_views"])
        return points
    
    elif camera["pose_method"] == "touch_look":
        points, vertex_list, vertex_normal_list = touch_look(obj_name="Icosphere")
        print("Length of points, vertex_list, vertex_normal_list are: ", len(points), len(vertex_list), len(vertex_normal_list))
        #print("Exiting the system...")
        #sys.exit()
        return points, vertex_list, vertex_normal_list
                           
    elif camera["pose_method"] == "local_look":
        points = local_look(radius=camera["circle_r"],height=camera["radius"],
                            phi=camera["phi"],theta=camera["theta"],psi=camera["psi"],
                            samples=camera["num_views"])
                            
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.set_aspect("equal")
        ax.scatter(points[0,:], points[1,:], points[2,:])
        ax.set_xlim(-camera["radius"], camera["radius"])
        ax.set_ylim(-camera["radius"],camera["radius"])
        ax.set_zlim(-camera["radius"],camera["radius"])
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        s_r = .11
        x = s_r*np.cos(u)*np.sin(v)
        y = s_r*np.sin(u)*np.sin(v)
        z = s_r*np.cos(v)
        ax.plot_surface(x, y, z, color="w", edgecolor="r")
        
        
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        
        #plt.show()
        plt.savefig(r"C:\Users\hp\AI\Stanford\ARMLab\NeRF_By_Touch\Blender\ARMLab Code - Tejas\NeRF Data\Cube with ridges\local_look.png")
        points = list(map(tuple, points.T))
                            
        return points

# generate camera positions around a sphere
def fibonacci_sphere(r=1., zcutoff=None, samples=1000):

    points = []
    phi = math.pi * (3. - math.sqrt(5.))  # golden angle in radians

    for i in range(samples):
        y = 1. - (i / float(samples - 1)) * 2.  # y goes from 1 to -1
        radius = math.sqrt(1. - y * y)  # radius at y

        theta = phi * i  # golden angle increment

        x = math.cos(theta) * radius
        z = math.sin(theta) * radius
        
        if zcutoff is not None:
            if r*z >= zcutoff:
                points.append((r*x, r*y, r*z))
        else:
            points.append((r*x, r*y, r*z))
    return points
 
 
# generate camera positions along surface of a cube
def cube_look(r=1., origin=np.array([0,0,0]), samples=1000):
    sample_per_face = int(np.ceil(samples/6))
    front_face_start = origin+np.array([r,0,0])
    left_face_start = origin+np.array([0,-r,0])
    back_face_start = origin+np.array([-r,0,0])
    right_face_start = origin+np.array([0,r,0])
    top_face_start = origin+np.array([0,0,r])
    bot_face_start = origin+np.array([0,0,-r])


    points = []
    xx = np.linspace(-r, r, int((np.sqrt(sample_per_face))))
    yy = np.linspace(-r, r, int((np.sqrt(sample_per_face))))
    print(xx.shape)

    for face in range(6):
     for i in xx:
        for j in yy:
            if face == 0:
                points.append(front_face_start + np.array([0,i,j]))
            elif face == 1:
                points.append(left_face_start + np.array([i,0,j]))
            elif face == 2:
                points.append(back_face_start + np.array([0,i,j]))
            elif face == 3:
                points.append(right_face_start + np.array([i,0,j]))
            elif face == 4:
                points.append(top_face_start + np.array([i,j,0]))
            elif face == 5:
                points.append(bot_face_start + np.array([i,j,0]))

    return points


# generate camera positions in the volume of a cone
def cone_look(radius=1,height=1,phi=0,theta=0,psi=0, samples=1000):
    r = np.linspace(0, radius, samples)
    alpha = np.linspace(0, 2*np.pi, samples)
    
    R, Alpha = np.meshgrid(r,alpha)
    R = R.reshape(-1)
    Alpha = Alpha.reshape(-1)
    #print(R.shape)
    ind = np.random.choice(np.arange(0,R.shape[0],1,dtype=int), size=samples, replace=False)
    R = R[ind]
    Alpha = Alpha[ind]
    
    z = np.linspace(0, height, samples)
    
    x = z*R*np.cos(Alpha)
    y = z*R*np.sin(Alpha)
    
    points = np.array([x,y,z])
    rx = Rx(theta)
    ry = Ry(phi)
    rz = Rz(psi)
    points = rz@ry@rx@points

    return points


# generate camera positions in the area of a circle centered on a point on a sphere
def local_look(radius=1,height=0.5,phi=0,theta=0,psi=0, samples=1000):
    r = np.linspace(0, radius, samples)
    alpha = np.linspace(0, -2*np.pi, samples)
    
    R, Alpha = np.meshgrid(r,alpha)
    R = R.reshape(-1)
    Alpha = Alpha.reshape(-1)
    #print(R.shape)
#    ind = np.random.randint(0,R.shape, size=samples)
    ind = np.random.choice(np.arange(0,R.shape[0],1,dtype=int), size=samples, replace=False)
    R = R[ind]
    Alpha = Alpha[ind]
    x = R*np.cos(Alpha)
    y = R*np.sin(Alpha)
    points = np.array([x,y,height*np.ones(x.shape)])
    rx = Rx(theta)
    ry = Ry(phi)
    rz = Rz(psi)
    points = rz@ry@rx@points

    return points

# to get the camera rotation based on the orientation of the surface normal
def camera_rotation(camera, DirectionNormal):
    rot_quat = DirectionNormal.to_track_quat('Z', 'Y')
    camera_rotation_euler = rot_quat.to_euler()
    return camera_rotation_euler


# specify and return changed camera look direction
def look_at(obj_camera, point):
    looking_direction = obj_camera.location - Vector(point)
    rot_quat = looking_direction.to_track_quat('Z', 'Y')
    obj_camera.rotation_euler = rot_quat.to_euler()


# to return the touch camera look direction and offset it
def touch_look_at(obj_camera, vertex_normal_list, index):
    # to get the vertex normal 
    vertex_normal = vertex_normal_list[index]
    camera_rotation_euler_vert = camera_rotation(camera, vertex_normal)
    obj_camera.rotation_euler = camera_rotation_euler_vert

    
    '''
    since the camera is already placed at the location (at vertex) we can make use of that location
    '''

    # get the current location of the camera
    location = obj_camera.location.copy()
    print("Camera Location: ", location)

    # calculate the distance from the origin
    distance = location.length

    print("Distance from Center: ", distance)

    # calculate the new distance from the origin
    new_distance = distance + offset_distance  # increase the distance by 0.001 units

    # normalize the location vector
    direction = location.normalized()

    # calculate the new location vector
    new_location = direction * new_distance
    print("New camera location: ", new_location)

    # set the new location of the camera
    obj_camera.location = new_location


# generate camera positions according to the surface normal
def touch_look(obj_name = 'Icosphere'):
    #bpy.context.scene.render.engine = 'CYCLES'
    #bpy.context.scene.view_layers["ViewLayer"].use_pass_z = True

    ## Get a reference to the object by name
    obj = bpy.data.objects[obj_name]

    print("Object selected is: ", obj)

    # Set the object as the active object
    bpy.context.view_layer.objects.active = obj

    # Select the object
    obj.select_set(True)

    # Get a reference to the imported object
    obj = bpy.context.active_object

    # to create a mesh to store the original cube data before subdividing
    bm_vert = bmesh.new()
    bm_vert.from_mesh(obj.data)

    # to capture the ORIGINAL VERTICES before the object is subdivided
    vertex_list = [vert for vert in bm_vert.verts]

    print("The totla number of vertices in the soccer ball are: ", len(vertex_list))

    # to offset the camera position
    vert_co_offset = []
    num_of_elements_in_list = 3
    for vert in vertex_list:
        camera_location = vert.co
        new_cam_location = [None] * num_of_elements_in_list
        for i in range(len(camera_location)):
            if camera_location[i] < 0:
                new_value = camera_location[i] - specified_offset_units
                new_cam_location[i] = new_value
            else:
                new_value = camera_location[i] + specified_offset_units
                new_cam_location[i] = new_value
        vert_co_offset.append(Vector(new_cam_location))

    
    # to get the vertex normals
    vertex_normal_list = []
    for vert_index in range(len(vertex_list)):
        vertex_normal = vertex_list[vert_index].normal
        vertex_normal_list.append(vertex_normal)

    # since we have 680 vertices in the soccer ball, returning the 100 random images
    random_indices = random.sample(range(0, 680), 200)

    new_vert_co_offset = []
    new_vertex_list = []
    new_vertex_normal_list = []
    
    # to use these random indices and generate new vert_co_offset, vertex_list, vertex_normal_list
    for i in random_indices:
        new_vert_co_offset.append(vert_co_offset[i])
        new_vertex_list.append(vertex_list[i])
        new_vertex_normal_list.append(vertex_normal_list[i])
    
    i = 0
    while i < 5:
        print("Vertex Location: ", new_vertex_list[i].co)
        i = i + 1
    
    return new_vert_co_offset, new_vertex_list, new_vertex_normal_list



# obtain corresponding depth image for the RGBD camera
def get_depth_image(cam_name=None, b_invert=False, save_fn=None):
    '''
    values 0 -> 255, 0 is furthest, 255 is closest
    assumes range map node maps from 0 to 1 values

    set b_invert to True if you want disparity image
    '''
    # print("Camera_name: ")
    # print(cam_name)
    # print("Exiting...")
    # sys.exit()

    if cam_name == "camera_2":
        #print("Have got a depth image with camera_2")
        #sys.exit()
        raw = np.asarray(bpy.data.images["Viewer Node"].pixels)
        scene = bpy.data.scenes['Scene']
        raw = np.reshape(raw, (scene.render.resolution_y, scene.render.resolution_x, 4))
        raw = raw[:, :, 0]
        raw = np.flipud(raw)

        depth0 = bpy.data.objects[cam_name].data.clip_start
        depth1 = bpy.data.objects[cam_name].data.clip_end
        
        raw = np.clip(raw,0,depth1)
        depth = raw
        img8 = (raw - depth0)/(depth1-depth0)*255
        #img8 = (raw - 0.35*depth1) / (fib_radius - 0.35*depth1) * 255
        img8 = img8.astype(np.uint8)

        if b_invert:
            depth = 1.0 / depth

        if not save_fn is None:
            if save_fn[-3:] == "npy":
    #            np.save(save_fn, depth)
                pth = save_fn[:-4]
                cv2.imwrite(pth+'.png', img8)
            else:
                cv2.imwrite(save_fn, depth)
                
        return depth
    
    # use the normal formula for the touch depth image
    else:
        raw = np.asarray(bpy.data.images["Viewer Node"].pixels)
        scene = bpy.data.scenes['Scene']
        raw = np.reshape(raw, (scene.render.resolution_y, scene.render.resolution_x, 4))
        raw = raw[:, :, 0]
        raw = np.flipud(raw)

        depth0 = bpy.data.objects[cam_name].data.clip_start
        depth1 = bpy.data.objects[cam_name].data.clip_end
        
        raw = np.clip(raw,0,depth1)
        depth = raw
        img8 = (raw - depth0)/(depth1-depth0)*255
        #img8 = (raw - 0.35*depth1) / (fib_radius - 0.35*depth1) * 255
        img8 = img8.astype(np.uint8)

        if b_invert:
            depth = 1.0 / depth

        if not save_fn is None:
            if save_fn[-3:] == "npy":
    #            np.save(save_fn, depth)
                pth = save_fn[:-4]
                cv2.imwrite(pth+'.png', img8)
            else:
                cv2.imwrite(save_fn, depth)
                
        return depth

# convert camera transform into a list for the json file
def listify_matrix(matrix):
    matrix_list = []
    for row in matrix:
        matrix_list.append(list(row))
    return matrix_list


    


if __name__ == "__main__":

    if not os.path.exists(fp):
        os.makedirs(fp)
    
    # generate file structure to save data
    for ctype in CAMERA_TYPE:
        if not os.path.exists(os.path.join(fp, ctype)):
             os.makedirs(os.path.join(fp,ctype))
             for type in TRAIN_TYPE:
                os.makedirs(os.path.join(fp,ctype, type))

    # generate N cameras with desired intrinsics
    # Possible Camera Pose Generation Methods: fibonacci_sphere, cube_look, cone_look, local_look
    cameras = [#{"name": "camera_1", "fl":50, "near":1e-4,
#                "far":0.025, "res_x": 600, "res_y": 480,
#                "radius": .115, "zcutoff": None, "num_views": 400,
#                "type":['color', 'depth'], "pose_method": "local_look",
#                "circle_r": .03, "phi": np.pi/14, "theta": np.pi/16,
#                "psi": 0,
#               },
#               {"name": "camera_1", "angle_x":160, "near":1e-4,
#                "far":0.015, "res_x": 600, "res_y": 600,
#                "radius": .115, "zcutoff": None, "num_views": 300,
#                "type":['color', 'depth'], "pose_method": "local_look",
#                "circle_r": .03, "phi": np.pi/14, "theta": np.pi/16,
#                "psi": 0,
#               },
               {"name": "camera_1", "angle_x":180, "fl_x":25, "near":0.0001,
                "far":0.025, "res_x": 1024, "res_y": 1024,
                "radius": .125, "zcutoff": None, "num_views": 10,
                "type":['touch'], "pose_method": "touch_look",
                "circle_r": .03, "phi": np.pi/14, "theta": np.pi/16,
                "psi": 0,
               },

               {"name": "camera_2", "angle_x":60, "near":.1,
                "far":1.5, "res_x": 1024, "res_y": 1024,
                "radius": 0.4, "zcutoff": None, "num_views": 10,
                "type":['depth'], "pose_method": "fibonacci_sphere",
               },
#
#               {"name": "camera_3", "angle_x":70, "near":1e-4,
#                "far":0.5, "res_x": 1024, "res_y": 1024,
#                "radius": .25, "zcutoff": None, "num_views": 300,
#                "type":['depth'], "pose_method": "fibonacci_sphere",
#               },
               
#               {"name": "camera_4", "angle_x":70, "near":1e-4,
#                "far":1, "res_x": 1024, "res_y": 1024,
#                "radius": .5, "zcutoff": None, "num_views": 300,
#                "type":['depth'], "pose_method": "fibonacci_sphere",
#               },
               
#               {"name": "camera_5", "angle_x":70, "near":1e-4,
#                "far":2, "res_x": 1024, "res_y": 1024,
#                "radius": 1, "zcutoff": None, "num_views": 50,
#                "type":['color', 'depth'], "pose_method": "fibonacci_sphere",
#               },
#
#               {"name": "camera_6", "angle_x":70, "near":1e-4,
#                "far":2, "res_x": 1024, "res_y": 1024,
#                "radius": 1.5, "zcutoff": None, "num_views": 50,
#                "type":['color', 'depth'], "pose_method": "fibonacci_sphere",
#               },

               {"name": "camera_7", "angle_x":30, "near":0.1,
                "far":1.5, "res_x": 1024, "res_y": 1024,
                "radius": 1, "zcutoff": None, "num_views": 10,
                "type":['color'], "pose_method": "fibonacci_sphere",
               },
              ]
              
#    try:
    # Specify Camera look point
    look_at_pt = (0., 0., 0.)
    
    # create camera objects
    for camera in cameras:
        camera_data = bpy.data.cameras.new(name=camera["name"])
        camera_object = bpy.data.objects.new(camera["name"], camera_data)
        bpy.context.scene.collection.objects.link(camera_object)
        bpy.data.objects[camera["name"]].select_set(True)
        
        if 'touch' in camera['type']:
            bpy.data.objects[camera["name"]].data.type = 'PANO'
            bpy.data.objects[camera["name"]].data.cycles.fisheye_lens = camera["fl_x"]
            bpy.data.objects[camera["name"]].data.cycles.fisheye_fov = np.deg2rad(camera["angle_x"])
        else:
            bpy.data.objects[camera["name"]].data.angle_x = np.deg2rad(camera["angle_x"])
 
        # print("HEY")
        # print("camera type: ", camera_object.data.type)
    
        
        bpy.data.objects[camera["name"]].data.clip_start = camera["near"]
        bpy.data.objects[camera["name"]].data.clip_end = camera["far"]
        bpy.data.objects[camera["name"]].select_set(False)

    # Background
    bpy.context.scene.render.dither_intensity = 0.0
    bpy.context.scene.render.film_transparent = True
    
    # Create collection for objects not to render with background
    objs = [ob for ob in bpy.context.scene.objects if ob.type in ('EMPTY') and 'Empty' in ob.name]
    #bpy.ops.object.delete({"selected_objects": objs})
    
    # get blender scene
    scene = bpy.context.scene
    out_data = {}

    if len(cameras) == 1:
        camera = cameras[0]
        for ctype in CAMERA_TYPE:
            out_data[ctype] = {}
            for type in TRAIN_TYPE:
                if ctype in camera["type"]:
                    out_data[ctype][type] = {}
                    out_data[ctype][type]["frames"] = []
                    if bpy.data.objects[camera["name"]].data.type == "PANO":
                        out_data[ctype][type]["camera_angle_x"] = np.deg2rad(camera["angle_x"]).item()
#                        out_data[ctype][type]["fl_x"] = camera["fl_x"]
                        
                    else:
                        out_data[ctype][type]["camera_angle_x"] =  bpy.data.objects[camera["name"]].data.angle_x, #2*math.atan(camera["res_x"] /(2*bpy.data.objects[camera["name"]].data.lens))
                    #out_data[ctype][type]["fl_x"] = bpy.data.objects[camera["name"]].data.lens
                    out_data[ctype][type]["near"] = bpy.data.objects[camera["name"]].data.clip_start
                    out_data[ctype][type]["far"] = bpy.data.objects[camera["name"]].data.clip_end
                    out_data[ctype][type]["types"] = camera["type"]
                    if camera["pose_method"] == "touch_look":
                        points, vertex_list, vertex_normal_list = gen_viewpoints(camera)
                    else:
                        points = gen_viewpoints(camera)
                    out_data[ctype][type]["points"] = points
                    out_data[ctype][type]["x_resolutions"] = [camera["res_x"]]*len(out_data[ctype][type]["points"])
                    out_data[ctype][type]["y_resolutions"] = [camera["res_y"]]*len(out_data[ctype][type]["points"])
                    out_data[ctype][type]["names"] = [camera["name"]]*len(out_data[ctype][type]["points"])
#                        out_data[ctype][type]["frames"] = []

    else:
        #print("Inside else statement as length of cameras is greater than 1")
        # keep track which camera types should be used (color, depth, touch)
        for ctype in CAMERA_TYPE:
            out_data[ctype] = {}
            for type in TRAIN_TYPE:
                # keep track of data needed for each training mode (train, test, val)
                out_data[ctype][type] = {}
                out_data[ctype][type]["cameras"] = {}
                out_data[ctype][type]["cameras"]
                out_data[ctype][type]["points"] = []
                out_data[ctype][type]["x_resolutions"] = []
                out_data[ctype][type]["y_resolutions"] = []
                out_data[ctype][type]["names"] = []
                out_data[ctype][type]["frames"] = []
                
                #generate entries for each camera and store them.
                for camera in cameras:
                    if ctype in camera["type"]:
                        cam = {
                                "w": camera["res_x"],
                                "h": camera["res_y"],
#                                "camera_angle_x": bpy.data.objects[camera["name"]].data.angle_x, #2*math.atan(camera["res_x"] /(2*bpy.data.objects[camera["name"]].data.lens)),
                                #"fl_x": bpy.data.objects[camera["name"]].data.lens,
                                "near": bpy.data.objects[camera["name"]].data.clip_start,
                                "far": bpy.data.objects[camera["name"]].data.clip_end,
                              }
                        if bpy.data.objects[camera["name"]].data.type == "PANO":
                            cam["camera_angle_x"] = np.deg2rad(camera["angle_x"]).item()
#                            cam["fl_x"] = camera["fl_x"]

                        else:
                            cam["camera_angle_x"] =  bpy.data.objects[camera["name"]].data.angle_x, #2*math.atan(camera["res_x"]
                            
                        out_data[ctype][type]["cameras"][camera["name"]] = cam
                        out_data[ctype][type]["cameras"][camera["name"]]['types'] = camera["type"]
                        # generate points depending upon the look
                        if camera["pose_method"] == "touch_look":
                            print("Inside touch look")
                            points, vertex_list, vertex_normal_list = gen_viewpoints(camera)
                        else:
                            points = gen_viewpoints(camera)
                        cam_points = points
                        out_data[ctype][type]["points"] = out_data[ctype][type]["points"] + cam_points
                        out_data[ctype][type]["x_resolutions"] = out_data[ctype][type]["x_resolutions"] + [camera["res_x"]]*len(cam_points)
                        out_data[ctype][type]["y_resolutions"] = out_data[ctype][type]["y_resolutions"] + [camera["res_y"]]*len(cam_points)
                        out_data[ctype][type]["names"] = out_data[ctype][type]["names"] + [camera["name"]]*len(cam_points)
                
                # shuffle points so there isn't positional bias in the data
                inds = np.arange(0, len(out_data[ctype][type]["x_resolutions"]))
                inds = np.random.permutation(inds)
                out_data[ctype][type]["points"] = list(np.asarray(out_data[ctype][type]["points"])[inds])
                out_data[ctype][type]["x_resolutions"] = list(np.asarray(out_data[ctype][type]["x_resolutions"])[inds])
                out_data[ctype][type]["y_resolutions"] = list(np.asarray(out_data[ctype][type]["y_resolutions"])[inds])
                out_data[ctype][type]["names"] = list(np.asarray(out_data[ctype][type]["names"])[inds])

    
    for ctype in CAMERA_TYPE:
        #print("CAMERA TYPE: ", ctype)
#        print("OI OI OI")
        for type in TRAIN_TYPE:
            #print("type")
#            print(out_data[ctype])
            if len(out_data[ctype][type]["points"]):
                points = out_data[ctype][type].pop("points")
                x_resolutions = out_data[ctype][type].pop("x_resolutions")
                y_resolutions = out_data[ctype][type].pop("y_resolutions")
                names = out_data[ctype][type].pop("names")
                
                for i, loc in enumerate(points):
                    #print("using camera: ", names[i])
                    #print("object type: ", bpy.data.objects[names[i]].data.type)

                    scene.render.resolution_x = x_resolutions[i]
                    scene.render.resolution_y = y_resolutions[i]
                    scene.render.resolution_percentage = 100
                    bpy.data.objects[names[i]].select_set(True)
                    cam = scene.objects[names[i]]
                    bpy.context.scene.camera = cam
                    cam.location = loc
                    #look_at(cam, look_at_pt)
                    
                    
                    
                    if bpy.data.objects[names[i]].data.type == "PANO":
                        #print("PANORAMIC")
                        bpy.context.scene.render.engine = 'CYCLES'

                        touch_look_at(cam, vertex_normal_list, i)

                        # print("focal length: ", bpy.data.objects[names[i]].data.cycles.fisheye_lens)
                        # print("field of view:", bpy.data.objects[names[i]].data.cycles.fisheye_fov)
                    else:
                        print("PRESPECTIVE")
                        bpy.context.scene.render.engine = 'BLENDER_EEVEE'
                        look_at(cam, look_at_pt)
                        # print("focal length: ", bpy.data.objects[names[i]].data.lens)
                        # print("field of view:", bpy.data.objects[names[i]].data.angle_x)
                    
                    if ctype == 'color':
                        scene.render.filepath = os.path.join(fp, ctype, type, 'r_' + str(i)) #fp + '/r_' + str(i)
                        bpy.ops.render.render(write_still=True)  # render still
                    elif ctype == 'depth' or 'touch':
                        scene.render.filepath = os.path.join(fp, ctype, type, 'r_' + str(i))
                        bpy.ops.render.render()
                        get_depth_image(cam_name=names[i], b_invert=False, save_fn=os.path.join(fp, ctype, type, 'r_' + str(i)+'.npy'))
                
                
                    frame_data = {
                                  'camera': names[i],
                                  'file_path': f"./train/r_{i}",
                                  'rotation': 0.1,
                                  'transform_matrix': listify_matrix(cam.matrix_world)
                    }
                    out_data[ctype][type]['frames'].append(frame_data)
                    gc.collect()
                
                
#                    if "types" in out_data[ctype][type]:
#                        print("HELLO THERE")
#                        print(out_data[ctype][type])
#                        del out_data[ctype][type]["types"]
#                    else:
#                        print("HELLO UM")
#                        for camera in cameras:
#                            del out_data[ctype][type]["cameras"][camera["name"]]["types"]
                    
                
                with open(os.path.join(fp, ctype, f'transforms_{type}.json'), 'w') as out_file:
                    json.dump(out_data[ctype][type], out_file, indent=4)
            print(" ")
        
    # Delete all cameras
    bpy.ops.object.select_all(action='DESELECT')
    for camera in cameras:
        if bpy.context.scene.objects.get(camera["name"]):
            bpy.data.objects[camera["name"]].select_set(True)
            bpy.ops.object.delete()












