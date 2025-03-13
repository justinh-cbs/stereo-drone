import numpy as np
import unittest

def track_features(frame_num, p0, p1):
    # Simulate feature tracking and generate the tracked positions
    tracked_positions = np.column_stack(((frame_num + 1) * np.ones(8), np.arange(8), np.arange(8)))

    if p0 is None:
        p0 = np.zeros((13, 4))
        p0[:, 0] = np.arange(13)

    if p1 is not None:
        p0_tracked = np.zeros_like(p0)
        p0_tracked[:, :3] = p0[:, :3]
        p0_tracked[p1[:, 0].astype(int), 1:3] = p1[:, 1:]
        p0 = p0_tracked

    p0[np.isnan(p0[:, 1]), 1:] = -1
    print(f"Frame num: {frame_num}")
    print(p0)

    return p0

class TestTrackFeatures(unittest.TestCase):
    def test_first_frame(self):
        frame_num = 0
        p0 = None
        p1 = None

        expected_output = np.zeros((13, 4))
        expected_output[:, 0] = np.arange(13)

        self.assertTrue(np.array_equal(track_features(frame_num, p0, p1), expected_output))

    def test_subsequent_frames(self):
        frame_num = 1
        p0 = np.zeros((13, 4))
        p0[:, 0] = np.arange(13)
        p1 = np.zeros((8, 2))
        p1[:, 0] = np.arange(8)
        p1[:, 1] = np.arange(8)

        expected_output = np.copy(p0)
        expected_output[p1[:, 0].astype(int), 1:3] = p1[:, 1:]

        self.assertTrue(np.array_equal(track_features(frame_num, p0, p1), expected_output))

    def test_new_points_added(self):
        frame_num = 2
        p0 = np.zeros((15, 4))
        p0[:, 0] = np.arange(15)
        p0[:8, 1] = np.arange(8)
        p1 = np.zeros((15, 2))
        p1[:8, 0] = np.arange(8)
        p1[:8, 1] = np.arange(8)
        p1[8:, 0] = np.arange(8, 15)
        p1[8:, 1] = np.arange(8, 15)

        expected_output = np.copy(p0)
        expected_output[p1[:, 0].astype(int), 1:3] = p1[:, 1:]

        self.assertTrue(np.array_equal(track_features(frame_num, p0, p1), expected_output))

if __name__ == '__main__':
    unittest.main()
