#!/usr/bin/env python

from geometry_msgs.msg import Pose

import numpy as np
from time import sleep
import math
import tf
import os

from choose import choose_from, choose_from_or_none
from paths import AGG_GRASPIT_DIR, RAW_GAZEBO_DIR, DATASET_TEMPLATE_PATH, MESH_MODELS_DIR
from date_string import get_date_string

from gazebo_model_manager import GazeboKinectManager, GazeboModelManager
from transformer_manager import TransformerManager
from xyz_to_pixel_loc import xyz_to_uv
from grasp_dataset import GraspDataset

from gazebo_output_aggregator import AggGazeboOutput

class GraspRGBDCapture():

    def __init__(self, model_path, input_graspit_dataset_full_filepath, gazebo_grasp_dataset_full_filepath, camera_dist=2):
        self.model_path = model_path
        self.input_graspit_dataset_full_filepath = input_graspit_dataset_full_filepath
        self.gazebo_grasp_dataset_full_filepath = gazebo_grasp_dataset_full_filepath

        self.camera_dist = camera_dist

    def build_camera_pose_in_grasp_frame(self, grasp, cameraDist):
        camera_pose = Pose()

        #this will back the camera off along the approach direction 'cameraDist' meters
        camera_pose.position.z -= cameraDist

        # quaternion = (grasp.pose.orientation.x, grasp.pose.orientation.y,grasp.pose.orientation.z,grasp.pose.orientation.w)
        # quaternion_inverse = tf.transformations.quaternion_inverse(quaternion)

        # r,p,y = tf.transformations.euler_from_quaternion(quaternion_inverse)

        #the camera points along the x direction, and we need it to point along the z direction
        roll = 0
        pitch = -math.pi/2.0

        # rotate so that camera is upright.
        yaw = 0#-math.pi/2.0 + grasp.joint_angles[0]

        # #rotation matrix from grasp to model rather than model to grasp.
        # quaternion_inverse = tf.transformations.quaternion_inverse(*grasp.pose.orientation)
        # q= quaternion_inverse

        # yaw = math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
        # #for intrinsic rotation
        # alpha, _, _ = tf.transformations.euler_from_quaternion(*quaternion_inverse,'szyx')

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        camera_pose.orientation.x = quaternion[0]
        camera_pose.orientation.y = quaternion[1]
        camera_pose.orientation.z = quaternion[2]
        camera_pose.orientation.w = quaternion[3]

        return camera_pose

    def calculate_palm_and_vc_image_locations(self, grasp_in_camera_frame, transform_manager, grasp):
        vc_uvds = []

        #this is the pixel location of the grasp point
        #u, v, d = xyz_to_uv((grasp_in_camera_frame.position.x, grasp_in_camera_frame.position.y, grasp_in_camera_frame.position.z))

        #vc_uvds.append((u, v, d))

        for i in range(len(grasp.virtual_contacts)):
            pose = Pose()
            pose.position.x = grasp.virtual_contacts[i][0]
            pose.position.y = grasp.virtual_contacts[i][1]
            pose.position.z = grasp.virtual_contacts[i][2]

            pose_in_world_frame = transform_manager.transform_pose(pose, "Model", "World").pose
            pose_in_camera_frame = transform_manager.transform_pose(pose, "Model", "Camera").pose

            u, v, d = xyz_to_uv((pose_in_camera_frame.position.x, pose_in_camera_frame.position.y, pose_in_camera_frame.position.z))
            #model_manager.spawn_sphere("sphere-%s-%s" % (graspNum, i),
            #                          pose_in_world_frame.position.x,
            #                          pose_in_world_frame.position.y,
            #                          pose_in_world_frame.position.z)
            vc_uvds.append((u, v, d))

        #sleep(1)

        return vc_uvds

    def update_transforms(self, transform_manager, grasp_pose, kinect_manager, cameraDist):
        transform_manager.add_transform(grasp_pose, "Model", "Grasp")

        camera_pose_in_grasp_frame = self.build_camera_pose_in_grasp_frame(grasp_pose, cameraDist)

        camera_pose_in_world_frame = transform_manager.transform_pose(camera_pose_in_grasp_frame, "Grasp", "World")
        grasp_pose_in_world_frame = transform_manager.transform_pose(grasp_pose, "Model", "World")

        #look at these as two points in world coords, ignoring rotations
        dx = camera_pose_in_world_frame.pose.position.x - grasp_pose_in_world_frame.pose.position.x
        dy = camera_pose_in_world_frame.pose.position.y - grasp_pose_in_world_frame.pose.position.y
        dz = camera_pose_in_world_frame.pose.position.z - grasp_pose_in_world_frame.pose.position.z

        #first find angle around world z to orient camera towards object
        rot = math.atan2(dy, dx)

        #now find angle to tilt camera down towards object
        dist_in_xy_plane = math.hypot(dx,dy)
        tilt = math.atan2(dz, dist_in_xy_plane)

        #now find rpy to rotate camera from 0,0,0,0 to rot, tilt
        roll = 0
        #make sure the camera is tilted up or down to center the palm vertically
        pitch = tilt
        #this centers the object in the x,y world plane. by rotating around world's z axis
        yaw = rot + math.pi

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        quat_grasp = camera_pose_in_world_frame.pose.orientation
        grasp_rpy = tf.transformations.euler_from_quaternion((quat_grasp.x, quat_grasp.y, quat_grasp.z, quat_grasp.w))
        camera_rpy = roll, pitch, yaw

        wrist_roll = grasp_rpy[0] - camera_rpy[0]

        camera_pose_in_world_frame.pose.orientation.x = quaternion[0]
        camera_pose_in_world_frame.pose.orientation.y = quaternion[1]
        camera_pose_in_world_frame.pose.orientation.z = quaternion[2]
        camera_pose_in_world_frame.pose.orientation.w = quaternion[3]

        #return false if the camera is below the XY plane (ie below the object)
        if camera_pose_in_world_frame.pose.position.z < 0:
            return False, False

        kinect_manager.set_model_state(camera_pose_in_world_frame.pose)

        transform_manager.add_transform(camera_pose_in_world_frame.pose, "World", "Camera")

        return True, wrist_roll

    def run(self):
        sleep(2)

        model_manager = GazeboModelManager(models_dir=self.model_path)
        model_manager.pause_physics()
        model_manager.clear_world()

        sleep(0.5)

        kinect_manager = GazeboKinectManager()

        if kinect_manager.get_model_state().status_message == 'GetModelState: model does not exist':
            kinect_manager.spawn_kinect()

        sleep(0.5)

        graspit_grasp_dataset = GraspDataset(self.input_graspit_dataset_full_filepath,
                                     DATASET_TEMPLATE_PATH + "/dataset_configs/graspit_grasps_dataset.yaml")

        num_iter = 0
        gazebo_grasp_dataset = GraspDataset(self.gazebo_grasp_dataset_full_filepath,
                                            DATASET_TEMPLATE_PATH + "/dataset_configs/gazebo_capture_config.yaml")

        model_names = os.listdir(self.model_path)
        model_name = None
        for grasp in graspit_grasp_dataset.iterator(start=num_iter):

            #if there is an old model, we need to remove it
            if model_name is not None and model_name != grasp.model_name[0]:
                model_manager.remove_model(model_name)

            #if there is no model, we need to spawn one.
            if model_name is None or model_name != grasp.model_name[0]:
                model_name = grasp.model_name[0]
                if model_name in model_names:
                    model_manager.spawn_model(model_name=model_name,  model_type=model_name)
                else:
                    model_name = None
                    continue

            grasp_pose = Pose()
            grasp_pose.position.x = grasp.palm_pose[0]
            grasp_pose.position.y = grasp.palm_pose[1]
            grasp_pose.position.z = grasp.palm_pose[2]
            grasp_pose.orientation.x = grasp.palm_pose[3]
            grasp_pose.orientation.y = grasp.palm_pose[4]
            grasp_pose.orientation.z = grasp.palm_pose[5]
            grasp_pose.orientation.w = grasp.palm_pose[6]

            transform_manager = TransformerManager()

            camera_pose_in_world_frame = model_manager.get_model_state(kinect_manager.camera_name).pose
            transform_manager.add_transform(camera_pose_in_world_frame, "World", "Camera")

            model_pose_in_world_frame = model_manager.get_model_state(model_name).pose
            transform_manager.add_transform(model_pose_in_world_frame, "World", "Model")

            print "%s: Running %s..." % (num_iter, model_name)

            #break so that we can manually add lights to gazebo
            if num_iter == 0:
                import pdb
                pdb.set_trace()

            updated_transforms, wrist_roll = self.update_transforms(transform_manager, grasp_pose, kinect_manager, self.camera_dist)

            if not updated_transforms:
                print "Camera below model... skipping this grasp"
                # go to next index if the camera is positioned below the object
            else:
                grasp_in_camera_frame = transform_manager.transform_pose(grasp_pose, "Model", "Camera").pose

                #vc_uvs is a list of (u,v) tuples in the camera_frame representing:
                #1)the palm,
                #2)all the virtual contacts used in graspit
                vc_uvds = self.calculate_palm_and_vc_image_locations(grasp_in_camera_frame, transform_manager, grasp)

                #this is a processed rgbd_image that has been normalized and any nans have been removed
                rgbd_image = np.copy(kinect_manager.get_normalized_rgbd_image())

                # Sometimes for no real reason openni fails and the depths stop being published.
                if rgbd_image[:, :, 3].max() == 0.0:
                    print "SOMETHING'S BROKEN!"
                    import IPython
                    IPython.embed()

                gazebo_grasp = gazebo_grasp_dataset.Grasp(
                    rgbd=rgbd_image,
                    dof_values=grasp.dof_values,
                    palm_pose=grasp.palm_pose,
                    joint_values=grasp.joint_values,
                    uvd=vc_uvds,
                    wrist_roll=wrist_roll,
                    virtual_contacts=grasp.virtual_contacts,
                    model_name=grasp.model_name,
                    energy=grasp.energy,
                    grasp_type=grasp.grasp_type
                )

                gazebo_grasp_dataset.add_grasp(gazebo_grasp)

            num_iter += 1

        #this calculates grasp priors, num_grasp_types and num_finger_types
        agg_gazebo_output = AggGazeboOutput(self.gazebo_grasp_dataset_full_filepath)
        agg_gazebo_output.run()

if __name__ == '__main__':

    graspit_agg_h5 = choose_from(AGG_GRASPIT_DIR)
    graspit_agg_name = graspit_agg_h5

    input_graspit_dataset_full_filepath = AGG_GRASPIT_DIR + graspit_agg_name

    gazebo_grasp_file = choose_from_or_none(RAW_GAZEBO_DIR)
    if not gazebo_grasp_file:
        gazebo_grasp_file = graspit_agg_name[:-3] + "-" + get_date_string() + ".h5"

    gazebo_grasp_dataset_full_filepath = RAW_GAZEBO_DIR + gazebo_grasp_file

    grasp_rgbd_capture = GraspRGBDCapture(model_path=MESH_MODELS_DIR + "big_bird_models_processed/",
                                          input_graspit_dataset_full_filepath=input_graspit_dataset_full_filepath,
                                          gazebo_grasp_dataset_full_filepath=gazebo_grasp_dataset_full_filepath)

    grasp_rgbd_capture.run()

    import IPython
    IPython.embed()

