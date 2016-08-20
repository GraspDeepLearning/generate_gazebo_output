#!/usr/bin/env python

from geometry_msgs.msg import Pose

import numpy as np
from time import sleep
import math
import tf
import os


import tf_conversions
import PyKDL
import scipy
from sensor_msgs.msg import Image, PointCloud2, PointCloud
from sensor_msgs import point_cloud2

from choose import choose_from, choose_from_or_none
from paths import AGG_GRASPIT_DIR, RAW_GAZEBO_DIR, DATASET_TEMPLATE_PATH, MESH_MODELS_DIR
from date_string import get_date_string

from gazebo_model_manager import GazeboKinectManager, GazeboModelManager


def save(pc, modelname, model_pose, r_index, p_index, y_index):
    data_dir = "gazebo_reconstruction_drill_yaw_only/" + modelname + "/"
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    id_string = "_" + str(r_index) + "_" + str(p_index) + "_" + str(y_index) + "_"
    pose_matrix = tf_conversions.toMatrix(tf_conversions.fromMsg(model_pose))

    np.save(data_dir + str(id_string) + "pc.npy", pc)
    np.save(data_dir + str(id_string) + "pose.npy", pose_matrix)


class GraspRGBDCapture():

    def __init__(self, model_path, camera_dist=1):
        self.model_path = model_path

        self.camera_dist = camera_dist

    def run(self):
        sleep(2)

        #model_manager = GazeboModelManager(models_dir=self.model_path)
        #model_manager.pause_physics()
        #model_manager.clear_world()

        sleep(0.5)

        kinect_manager = GazeboKinectManager(use_pc=True)

        #spawn the kinect
        f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(-self.camera_dist, 0, 0))
        kinect_pose = tf_conversions.posemath.toMsg(f)

        if kinect_manager.get_model_state().status_message == 'GetModelState: model does not exist':
            kinect_manager.spawn_kinect(model_pose=kinect_pose)


        import IPython
        IPython.embed()
        model_names = os.listdir(self.model_path)
        model_names = ["cordless_drill"]
        model_names = ["cessna"]

        first_time = True
        for model_name in model_names:

            #spawn the model
            # f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, 0, 0))
            # model_pose = tf_conversions.posemath.toMsg(f)
            # model_manager.spawn_model(model_name=model_name,  model_type=model_name, model_pose=model_pose)
            model_manager.spawn_model(model_name=model_name,  model_type=model_name)

            sleep(0.5)

            if first_time:
                import pdb
                pdb.set_trace()
                first_time = False

            step_size = .6
            step_size = .02

            rs = [0]#np.arange(0, 2*math.pi, step_size)
            ps = [0]#np.arange(-math.pi/2.0, math.pi/2.0, step_size)
            ys = np.arange(0, 2*math.pi, step_size)
            print len(ys)

            for r_index in range(len(rs)):
                for p_index in range(len(ps)):
                    for y_index in range(len(ys)):

                        r = rs[r_index]
                        p = ps[p_index]
                        y = ys[y_index]

                        f = PyKDL.Frame(PyKDL.Rotation.RPY(r, p, y), PyKDL.Vector(0, 0, 0))
                        model_pose = tf_conversions.posemath.toMsg(f)

                        #set the model to a new pose
                        model_manager.set_model_state(model_name, model_pose)

                        sleep(.4)

                        save(kinect_manager.get_processed_point_cloud(),
                             model_name,
                             model_pose,
                             r_index,
                             p_index,
                             y_index)

            model_manager.remove_model(model_name)

if __name__ == '__main__':

    model_path = MESH_MODELS_DIR + "big_bird_models_processed"
    #model_path = "/home/jared/reconstruction_models/gazebo"
    model_path = "/home/avinash/.gazebo/models"
    #~/gdl_ws/src/data/data/mesh_models/big_bird_models_processed/

    grasp_rgbd_capture = GraspRGBDCapture(model_path)

    grasp_rgbd_capture.run()

    import IPython
    IPython.embed()

