import numpy as np

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

    return p0

def main():
    p0 = None

    for frame_num in range(3):
        p1 = None

        if frame_num > 0:
            p1 = np.zeros((8, 2))
            p1[:, 0] = np.arange(8)

        p0 = track_features(frame_num, p0, p1)
        print(f"Frame {frame_num + 1}:")
        print(p0)
        print()

if __name__ == '__main__':
    main()
