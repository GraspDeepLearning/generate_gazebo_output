import h5py
import numpy as np

from choose import choose_from, choose_from_or_none
from paths import CONDENSED_GAZEBO_DIR, GRASP_PRIORS_DIR

class AggGazeboOutput():

    def __init__(self, in_filepath):
        self.in_filepath = in_filepath

    def run(self):

        dset = h5py.File(self.in_filepath)

        wrist_roll = dset['wrist_roll']
        uvd = dset['uvd']
        joint_values = dset['joint_values']

        num_grasp_types = np.max(dset['grasp_type']) + 1

        if not 'avg_wrist_roll' in dset.keys():
            dset.create_dataset('num_grasp_type', shape=(1,))
            dset.create_dataset('num_finger_type', shape=(1,))
            dset.create_dataset('avg_wrist_roll', shape=(num_grasp_types, 1))
            dset.create_dataset('avg_joint_values', shape=(num_grasp_types, 8))
            dset.create_dataset('avg_uvd', shape=(num_grasp_types, 4, 3))

        grasp_types_count = np.zeros(num_grasp_types)

        avg_wrist_roll = np.zeros((num_grasp_types, 1))
        avg_uvd = np.zeros((num_grasp_types, 4, 3))
        avg_joint_values = np.zeros((num_grasp_types, 8))

        for i in range(len(dset["grasp_type"])):
            grasp_type = dset["grasp_type"][i][0]
            grasp_types_count[grasp_type] += 1

            avg_wrist_roll[grasp_type] += wrist_roll[i]
            avg_uvd[grasp_type] += uvd[i]
            avg_joint_values[grasp_type] += joint_values[i]

        for i in range(len(grasp_types_count)):
            avg_wrist_roll[i] /= grasp_types_count[i]
            avg_uvd[i] /= grasp_types_count[i]
            avg_joint_values[i] /= grasp_types_count[i]

        dset['num_grasp_type'][0] = dset['grasp_type'][:].max() + 1
        dset['num_finger_type'][0] = 4
        dset['avg_wrist_roll'][:] = avg_wrist_roll
        dset['avg_uvd'][:] = avg_uvd
        dset['avg_joint_values'][:] = avg_joint_values

if __name__ == "__main__":

    # Choose in and out files.
    in_file = choose_from(CONDENSED_GAZEBO_DIR)
    in_filepath = CONDENSED_GAZEBO_DIR + in_file

    gpl_generator = AggGazeboOutput(in_filepath=in_filepath)
    gpl_generator.run()

