#!/usr/bin/env python

from geometry_msgs.msg import Pose

import numpy as np
from time import sleep
import math
import tf
import os
import pcl

import tf_conversions
import PyKDL
import scipy
from sensor_msgs.msg import Image, PointCloud2, PointCloud
from sensor_msgs import point_cloud2

from choose import choose_from, choose_from_or_none
from paths import RAW_GAZEBO_DIR, DATASET_TEMPLATE_PATH, MESH_MODELS_DIR, AGG_GRASPIT_DIR
from date_string import get_date_string

from gazebo_model_manager import GazeboKinectManager, GazeboModelManager


class GraspRGBDCapture():

    def __init__(self, model_path, camera_dist=1):
        self.model_path = model_path
        self.camera_dist = camera_dist
        self.date_str = get_date_string()

    def run(self):
        sleep(2)

        model_manager = GazeboModelManager(models_dir=self.model_path)
        #model_manager.pause_physics()
        model_manager.clear_world()

        sleep(0.5)

        kinect_manager = GazeboKinectManager(use_pc=True)

        #spawn the kinect
        f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(-self.camera_dist, 0, 0))
        kinect_pose = tf_conversions.posemath.toMsg(f)

        if kinect_manager.get_model_state().status_message == 'GetModelState: model does not exist':
            kinect_manager.spawn_kinect(model_pose=kinect_pose)

        #model_names = os.listdir(self.model_path)
        # model_names = []
        # model_names.append("rubbermaid_ice_guard_pitcher_blue")
        # model_names.append("brine_mini_soccer_ball")
        # model_names.append("block_of_wood_12in")
        # model_names.append("block_of_wood_6in")
        # model_names.append("domino_sugar_1lb")
        # model_names.append("cheerios_14oz")
        # model_names.append("cheeze-it_388g")
        # model_names.append("clorox_disinfecting_wipes_35")
        # model_names.append("comet_lemon_fresh_bleach")
        # model_names.append("spam_12oz")
        # model_names.append("pringles_original")

        model_names = ['black_and_decker_lithium_drill_driver',
        'blue_wood_block_1inx1in',
        'brine_mini_soccer_ball',
        'campbells_condensed_tomato_soup',
        'clorox_disinfecting_wipes_35',
        'comet_lemon_fresh_bleach',
        'domino_sugar_1lb',
        'frenchs_classic_yellow_mustard_14oz',
        'melissa_doug_farm_fresh_fruit_lemon',
        'morton_salt_shaker',
        'play_go_rainbow_stakin_cups_1_yellow',
        'pringles_original',
        'rubbermaid_ice_guard_pitcher_blue',
        'soft_scrub_2lb_4oz',
        'sponge_with_textured_cover',
        'block_of_wood_6in',
        'cheerios_14oz',
        'melissa_doug_farm_fresh_fruit_banana',
        'play_go_rainbow_stakin_cups_2_orange']

        print model_names
        have_set_cameras_in_gazebo = False
        for model_name in model_names:

            model_manager.spawn_model(model_name=model_name,  model_type=model_name)

            sleep(0.5)

            if not have_set_cameras_in_gazebo:
                import pdb
                pdb.set_trace()
                have_set_cameras_in_gazebo = True

            step_size = .6

            rs = np.arange(0, 2*math.pi, step_size)
            ps = np.arange(-math.pi/2.0, math.pi/2.0, step_size)
            ys = np.arange(0, 2*math.pi, step_size)
            print len(ys)

            old_pc = None
            first_time = True
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

                        sleep(.02)

                        pc = kinect_manager.get_processed_point_cloud()
                        rgbd = kinect_manager.get_normalized_rgbd_image()
                        if old_pc != None:
                            for idx in range(30):
                                if old_pc.shape == pc.shape:
                                    print "GETTING NEW CLOUD, THIS IS SAME AS OLD" + str(idx)
                                    pc = kinect_manager.get_processed_point_cloud()
                                    rgbd = kinect_manager.get_normalized_rgbd_image()
                                else:
                                    break

                        if first_time:
                            sleep(1.0)
                            first_time = False
                            pc = kinect_manager.get_processed_point_cloud()
                            rgbd = kinect_manager.get_normalized_rgbd_image()
                            if old_pc != None:
                                for idx in range(30):
                                    if old_pc.shape == pc.shape:
                                        print "GETTING NEW CLOUD, THIS IS SAME AS OLD" + str(idx)
                                        pc = kinect_manager.get_processed_point_cloud()
                                        rgbd = kinect_manager.get_normalized_rgbd_image()
                                    else:
                                        break

                        old_pc = np.copy(pc)
                        self.save(pc,
                                  rgbd,
                                  model_name,
                                  model_pose,
                                  kinect_pose,
                                  r_index,
                                  p_index,
                                  y_index)
                        break
                    break
                break

            model_manager.remove_model(model_name)

    def save(self, pc, rgbd,  modelname, model_pose, camera_pose, r_index, p_index, y_index):
        data_dir = "/srv/data/gazebo_generated/data/ycb/" + self.date_str + "/" + modelname + "/pointclouds/"

        if not os.path.exists(data_dir):
            os.makedirs(data_dir)

        id_string = "_" + str(r_index) + "_" + str(p_index) + "_" + str(y_index) + "_"
        pose_matrix = tf_conversions.toMatrix(tf_conversions.fromMsg(model_pose))
        camera_matrix = tf_conversions.toMatrix(tf_conversions.fromMsg(camera_pose))

        #np.save(data_dir + str(id_string) + "pc.npy", pc)
        pcd = pcl.PointCloud(np.array(pc[:, 0:3],np.float32))
        pcl.save(pcd, data_dir + str(id_string) + "pc.pcd")
        np.save(data_dir + str(id_string) + "rgbd.npy", rgbd)
        np.save(data_dir + str(id_string) + "model_pose.npy", pose_matrix)
        np.save(data_dir + str(id_string) + "camera_pose.npy", camera_matrix)

if __name__ == '__main__':

    model_path = "/srv/data/processed_mesh_models/ycb/"

    grasp_rgbd_capture = GraspRGBDCapture(model_path)
    grasp_rgbd_capture.run()


