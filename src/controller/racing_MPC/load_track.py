import numpy as np
import os
from pathlib import Path

def load_track(filename):
    data_path = os.path.join(str(Path(__file__).parent), filename)
    track_data = np.loadtxt(data_path, delimiter=',')
    return track_data
