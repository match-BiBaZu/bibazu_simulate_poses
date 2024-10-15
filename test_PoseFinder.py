import unittest
import numpy as np
from pathlib import Path
from bibazu_simulate_poses.PoseFinder import PoseFinder

class TestPoseFinder(unittest.TestCase):

    def setUp(self):
        self.pose_finder = PoseFinder()
        self.pose_finder.data_path = Path('/path/to/test/data')  # Update this path to your test data directory
        self.pose_finder.workpiece_name = 'test_workpiece'
        self.pose_finder.simulation_number = 1000
        self.pose_finder.array_location = np.zeros((self.pose_finder.simulation_number, 3))
        self.pose_finder.array_rotation_blend = np.zeros((self.pose_finder.simulation_number, 4))
        self.pose_finder.array_quaternion_blend = np.zeros((self.pose_finder.simulation_number, 5))

    def test_import_orientation_csv(self):
        # Create mock data files
        location_data = np.random.rand(self.pose_finder.simulation_number, 3)
        rotation_data = np.random.rand(self.pose_finder.simulation_number, 3)
        quaternion_data = np.random.rand(self.pose_finder.simulation_number, 4)

        np.savetxt(self.pose_finder.data_path / (self.pose_finder.workpiece_name + '_simulated_data_export_matlab_location.txt'), location_data)
        np.savetxt(self.pose_finder.data_path / (self.pose_finder.workpiece_name + '_simulated_data_export_matlab_rotation.txt'), rotation_data)
        np.savetxt(self.pose_finder.data_path / (self.pose_finder.workpiece_name + '_simulated_data_export_matlab_quaternion.txt'), quaternion_data)

        # Call the method to test
        self.pose_finder.import_orientation_csv()

        # Check if the data was imported correctly
        np.testing.assert_array_almost_equal(self.pose_finder.array_location, location_data)
        np.testing.assert_array_almost_equal(self.pose_finder.array_rotation_blend[:, :3], rotation_data * np.pi / 180)
        np.testing.assert_array_almost_equal(self.pose_finder.array_quaternion_blend[:, :4], quaternion_data)

if __name__ == '__main__':
    unittest.main()