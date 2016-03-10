#!/usr/bin/env python

"""
Calculates a disparity map from two video feeds, undistorting videos with
calibration data in '../data/'
Project: TeenBot Disparity Map
Author: Eelis Peltola
Started: 11.03.2016
Last modified: 11.03.2016
"""

from cv2 import *
import numpy as np
import yaml


def nothing(x):
    pass


def calculate_maps(image_size):
    with open("../data/instrinsics.yml", 'r') as intr_stream:
        [cm1, cameramatrix_l, d1, distcoeffs_l, cm2,
         cameramatrix_r, d2, distcoeffs_r] = yaml.load_all(intr_stream)

    with open("../data/extrinsics.yml", 'r') as extr_stream:
        [r1, RL, r2, RR, p1, PL, p2, PR, q, Q] = yaml.load_all(extr_stream)

    mapl_x, mapl_y = initUndistortRectifyMap(cameramatrix_l, distcoeffs_l, RL,
                                             PL, image_size, CV_16SC2, None,
                                             None)

    mapr_x, mapr_y = initUndistortRectifyMap(cameramatrix_r, distcoeffs_r, RR,
                                             PR, image_size, CV_16SC2, None,
                                             None)
    return mapl_x, mapl_y, mapr_x, mapr_y


