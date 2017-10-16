#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Pose
from gazebo_ros import gazebo_interface
from gazebo_msgs.srv import (DeleteModelRequest, DeleteModelResponse, DeleteModel, 
                             GetModelState, GetModelStateRequest, GetWorldProperties,
                             SetModelState, SetModelStateRequest)

import numpy as np
import os
from time import sleep

import tf_conversions
import tf
import PyKDL
import math
import std_srvs.srv

import time
import threading
import copy

rospack = rospkg.RosPack()


class RGBDListener():

    def __init__(self,
                 depth_topic="/camera1/camera/depth/image_raw",
                 rgb_topic="/camera1/camera/rgb/image_raw",
                 pc_topic="/camera1/camera/depth/points",
                 use_pc=False):

        self.depth_topic = depth_topic
        self.rgb_topic = rgb_topic
        self.pc_topic = pc_topic
        self.use_pc = use_pc
        self._rgbd_image = np.zeros((480, 640, 4))
        self._pc = np.zeros((10, 3))
        #self.last_update_time = time.
        self.semaphore  = threading.Semaphore()  
        
    def depth_image_callback(self, data):
        print "in depth callback"
        self.semaphore.acquire()
        depth_image_np = self.image2numpy(data)
        print "got depth with max val: " + str(depth_image_np.max())
        self._rgbd_image[:, :, 3] = depth_image_np
        self.semaphore.release()
        
    def rgb_image_callback(self, data):
        print "in rgb callback"
        self.semaphore.acquire()
        rgbd_image_np = self.image2numpy(data)
        print "got rgb with max val: " + str(rgbd_image_np.max())
        self._rgbd_image[:, :, 0:3] = rgbd_image_np
        self.semaphore.release()

    def set_pc(self, msg):
        self.semaphore.acquire()
        #print "Got a new pointcloud !"
        points = point_cloud2.read_points(msg)
        points_list = np.asarray(list(points))
        points_arr = np.asarray(points_list)
        self._pc =  np.copy(points_arr[~np.isnan(points_arr).any(axis=1)])
        self.semaphore.release()

    def clear_pc(self):
        self.semaphore.acquire()
        self._pc= None
        self.semaphore.release()

    def get_pc(self):
        self.semaphore.acquire()
        out_pc = None
        if self._pc != None:
            out_pc =  np.copy(self._pc)
        self.semaphore.release()
        return out_pc

    def get_rgbd(self):
        self.semaphore.acquire()
        out_rgbd = None
        if self._pc != None:
            out_rgbd =  np.copy(self._rgbd_image)
        self.semaphore.release()
        return out_rgbd


    #this method from:
    #https://github.com/rll/sushichallenge/blob/master/python/brett2/ros_utils.py
    def image2numpy(self, image):
        if image.encoding == 'rgb8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width, 3)[:, :, ::-1]
        if image.encoding == 'bgr8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width, 3)
        elif image.encoding == 'mono8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width)
        elif image.encoding == '32FC1':
            return np.fromstring(image.data, dtype=np.float32).reshape(image.height, image.width)
        else:
            raise Exception

    def listen(self):

        rospy.init_node('listener', anonymous=True)

        if self.use_pc:
            print "Using Point Cloud"
            rospy.Subscriber(self.pc_topic, PointCloud2, self.set_pc, queue_size=1)
        #else:
            rospy.Subscriber(self.depth_topic, Image, self.depth_image_callback, queue_size=1)
            rospy.Subscriber(self.rgb_topic, Image, self.rgb_image_callback, queue_size=1)


class GazeboKinectManager():
    def __init__(self, gazebo_namespace="/gazebo", use_pc=False):
        self.gazebo_namespace = gazebo_namespace
        self.rgbd_listener = RGBDListener(use_pc=use_pc)
        self.rgbd_listener.listen()
        self.camera_name = "camera1"

        self.get_model_state_service = rospy.ServiceProxy(gazebo_namespace + '/get_model_state', GetModelState)
        self.set_model_state_service = rospy.ServiceProxy(gazebo_namespace + '/set_model_state', SetModelState)

    def spawn_kinect(self, model_pose=None):
        model_xml = rospy.get_param("robot_description")
        if model_pose is None:
            #f = PyKDL.Frame(PyKDL.Rotation.RPY(0, math.pi, math.pi), PyKDL.Vector(0, 0, 2))
            f = PyKDL.Frame(PyKDL.Rotation.RPY(0, math.pi+math.pi/4.0, math.pi), PyKDL.Vector(0, 0, 2))
            f = PyKDL.Frame(PyKDL.Rotation.RPY(0, math.pi/4.0, 0), PyKDL.Vector(0, 0, 2))
            model_pose = tf_conversions.posemath.toMsg(f)
        robot_namespace = self.camera_name
        gazebo_interface.spawn_urdf_model_client(model_name=self.camera_name,
                                                model_xml=model_xml,
                                                robot_namespace=robot_namespace,
                                                initial_pose=model_pose,
                                                reference_frame="world",
                                                gazebo_namespace=self.gazebo_namespace)

    def get_normalized_rgbd_image(self):
        rgbd_image = np.copy(self.rgbd_listener.get_rgbd())
        while rgbd_image == None:
            rgbd_image = self.rgbd_listener.get_rgbd()
            time.sleep(0.01)
            print "Getting the pointcloud still"
        
        #fix nans in depth
        max_depth = np.nan_to_num(rgbd_image[:, :, 3]).max()*1.3 + 5.0
        for x in range(rgbd_image.shape[0]):
            for y in range(rgbd_image.shape[1]):
                if rgbd_image[x, y, 3] != rgbd_image[x, y, 3]:
                    rgbd_image[x, y, 3] = max_depth

        return rgbd_image

    def get_processed_point_cloud(self):
        self.rgbd_listener.clear_pc()
        pc = self.rgbd_listener.get_pc()
        while pc == None:
            pc = self.rgbd_listener.get_pc()
            time.sleep(0.01)
            print "Getting the pointcloud still"
        return pc

    def get_model_state(self):
        get_model_state_req = GetModelStateRequest()
        get_model_state_req.model_name = self.camera_name
        return self.get_model_state_service(get_model_state_req)

    def set_model_state(self, pose=Pose()):
        set_model_state_req = SetModelStateRequest()
        set_model_state_req.model_state.model_name = self.camera_name
        set_model_state_req.model_state.pose = pose
        return self.set_model_state_service(set_model_state_req)



class GazeboModelManager():

    def __init__(self,
                 models_dir,
                 gazebo_namespace="/gazebo"):

        self.gazebo_namespace = gazebo_namespace
        self.models_dir = models_dir
        self.delete_model_service_proxy = rospy.ServiceProxy(gazebo_namespace + '/delete_model', DeleteModel)
        self.get_model_state_service_proxy = rospy.ServiceProxy(gazebo_namespace + '/get_model_state', GetModelState)
        self.set_model_state_service_proxy = rospy.ServiceProxy(gazebo_namespace + '/set_model_state', SetModelState)
        self.get_world_properties_proxy = rospy.ServiceProxy(gazebo_namespace + '/get_world_properties', GetWorldProperties)
        self.pause_physics_service_proxy = rospy.ServiceProxy(gazebo_namespace + "/pause_physics", std_srvs.srv.Empty)
        self.unpause_physics_service_proxy = rospy.ServiceProxy(gazebo_namespace + "/unpause_physics", std_srvs.srv.Empty)

    def remove_model(self, model_name="coke_can"):

        del_model_req = DeleteModelRequest(model_name)
        self.delete_model_service_proxy(del_model_req)

    def clear_world(self):
        world_properties = self.get_world_properties_proxy()
        for model_name in world_properties.model_names:
            if model_name != "camera1":
                self.remove_model(model_name)

    def pause_physics(self):
        self.pause_physics_service_proxy()

    def unpause_physics(self):
        self.unpause_physics_service_proxy()

    def spawn_model(self, model_name="coke_can", model_type="coke_can", model_pose=None):
        model_xml = open(self.models_dir + "/" + model_type + "/model.sdf").read()
        if not model_pose:
            model_pose = Pose()
            model_pose.position.x = 0
            model_pose.position.y = 0
            model_pose.position.z = 0
        robot_namespace = model_name
        gazebo_interface.spawn_sdf_model_client(model_name=model_name,
                                                model_xml=model_xml,
                                                robot_namespace=robot_namespace,
                                                initial_pose=model_pose,
                                                reference_frame="world",
                                                gazebo_namespace=self.gazebo_namespace)

        #large models can take a moment to load
        while not self.does_world_contain_model(model_name):
            sleep(0.5)

    def spawn_sphere(self, sphere_name="0", x=0, y=0, z=0):

        model_xml = open(os.environ["GDL_PATH"] + "/models/sphere/model.sdf").read()
        model_pose = Pose()
        model_pose.position.x = x
        model_pose.position.y = y
        model_pose.position.z = z
        robot_namespace = sphere_name
        gazebo_interface.spawn_sdf_model_client(model_name=sphere_name,
                                                model_xml=model_xml,
                                                robot_namespace=robot_namespace,
                                                initial_pose=model_pose,
                                                reference_frame="world",
                                                gazebo_namespace=self.gazebo_namespace)


    def does_world_contain_model(self, model_name="coke_can"):
        get_model_state_req = GetModelStateRequest()
        get_model_state_req.model_name = model_name
        resp = self.get_model_state_service_proxy(get_model_state_req)
        return resp.success

    def get_model_state(self, model_name="coke_can"):
        get_model_state_req = GetModelStateRequest()
        get_model_state_req.model_name = model_name
        return self.get_model_state_service_proxy(get_model_state_req)

    def set_model_state(self, model_name="coke_can", pose=Pose()):

        set_model_state_req = SetModelStateRequest()
        set_model_state_req.model_state.model_name = model_name
        set_model_state_req.model_state.pose = pose
        result =  self.set_model_state_service_proxy(set_model_state_req)

        resp = self.get_model_state(model_name)

        p0 = tf_conversions.fromMsg(pose)
        p1 = tf_conversions.fromMsg(resp.pose)
        while not tf_conversions.Equal(p0, p1):
            print pose
            print resp.pose
            resp = self.get_model_state(model_name)
            p1 = tf_conversions.fromMsg(resp.pose)
            sleep(0.01)
            print "still updating model state"
        print "Good to go"


