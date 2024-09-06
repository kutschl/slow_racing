import os
import numpy as np


def import_track(file_name):
    root_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(root_dir, "tracks", file_name)

    track_data = np.loadtxt(file_path, comments='#', delimiter=',')
    fill1 = np.full((track_data.shape[0], 1), 2.5)
    fill2 = np.full((track_data.shape[0], 1), 2.5)
    track_data = np.hstack((track_data, fill1, fill2))

    return track_data
