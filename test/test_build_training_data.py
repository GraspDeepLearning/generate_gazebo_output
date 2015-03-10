
import unittest
import os
from paths import TEST_AGG_GRASPIT_DIR, TEST_RAW_GAZEBO_DIR, MESH_MODELS_DIR

from build_training_data import GraspRGBDCapture


class TestGraspRGBDCapture(unittest.TestCase):

    def setUp(self):

        self.input_graspit_dataset_full_filepath = TEST_AGG_GRASPIT_DIR + "contact_and_potential_grasps.h5"
        self.gazebo_grasp_dataset_full_filepath = TEST_RAW_GAZEBO_DIR + "contact_and_potential_grasps.h5"

        self.grasp_rgbd_capture = GraspRGBDCapture(model_path=MESH_MODELS_DIR + "big_bird_models_processed",
                                              input_graspit_dataset_full_filepath=self.input_graspit_dataset_full_filepath,
                                              gazebo_grasp_dataset_full_filepath=self.gazebo_grasp_dataset_full_filepath)

    def test_image_capture(self):
        self.grasp_rgbd_capture.run()

    #def tearDown(self):
        #if os.path.exists(self.gazebo_grasp_dataset_full_filepath):
        #    os.remove(self.gazebo_grasp_dataset_full_filepath)

if __name__ == '__main__':
    unittest.main()