#!/usr/bin/env python

"""
Stereo camera calibration for two webcams with chess board.
Reads uncalibrated images, calculates calibration matrices and writes
calibrated images.
Project: TeenBot Disparity Map
Author: Eelis Peltola
Started: 07.03.2016
Last modified: 07.03.2016
"""

import cv2
import yaml
import numpy as np
from glob import glob


__author__ = "Eelis Peltola"
__project__ = "Stereo calibration"
__version__ = "1.0beta"


stereo_imgmask_l = "../data/stereo_calib/left*.png"
stereo_imgnames_l = sorted(glob(stereo_imgmask_l))

stereo_imgmask_r = "../data/stereo_calib/right*.png"
stereo_imgnames_r = sorted(glob(stereo_imgmask_r))

if len(stereo_imgnames_l) != len(stereo_imgnames_r):
    print("Unidentical number of left and right images.")

boardW = 7
boardH = 5
squareSize = 30.0

boardSize = (boardW, boardH)
patternPoints = np.zeros((np.prod(boardSize), 3), np.float32)
patternPoints[:, :2] = np.indices(boardSize).T.reshape(-1, 2)
patternPoints *= squareSize
corner_wait_time = 5


img_mask_l = "../data/left_calib/left*.png"
imnl = sorted(glob(img_mask_l))

img_mask_r = "../data/right_calib/right*.png"
imnr = sorted(glob(img_mask_r))
