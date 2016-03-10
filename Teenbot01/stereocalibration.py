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


# TODO: Take out chessboard finding to its own function
# TODO: Convert to only calibrate one set of frames at a time, instead of two
def calib(img_names_l, img_names_r, board_size, pattern_points):
    obj_points = []
    img_points_l = []
    img_points_r = []
    img_size = cv2.imread(img_names_l[0], 0).shape[:2]
    h, w = (0, 0)

    cv2.namedWindow("left corners")
    cv2.namedWindow("right corners")

    print("Locating chessboard corners in images \n    {} and \n    {}"
          .format(img_mask_l, img_mask_r))

    for fn_l, fn_r in zip(img_names_l, img_names_r):
        print("Reading {} and {}... ".format(fn_l.split('/')[-1],
                                             fn_r.split('/')[-1]), end='')
        img_l = cv2.imread(fn_l, 0)
        img_r = cv2.imread(fn_r, 0)

        if img_l is None:
            print("failed to load ", fn_l)
            continue

        if img_r is None:
            print("failed to load ", fn_r)
            continue

        if img_l.shape[:2] != img_size or img_r.shape[:2] != img_size:
            print("dimensions of {} or {} do not match".format(fn_l, fn_r))
            continue

        h, w = img_l.shape[:2]
        found_l, corners_l =\
            cv2.findChessboardCorners(img_l, board_size,
                                      cv2.CALIB_CB_ADAPTIVE_THRESH |
                                      cv2.CALIB_CB_NORMALIZE_IMAGE)
        found_r, corners_r =\
            cv2.findChessboardCorners(img_r, board_size,
                                      cv2.CALIB_CB_ADAPTIVE_THRESH |
                                      cv2.CALIB_CB_NORMALIZE_IMAGE)

        if found_l:
            term_c = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(img_l, corners_l, (5, 5), (-1, -1), term_c)
            vis_l = cv2.cvtColor(img_l, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(vis_l, board_size, corners_l, found_l)
            cv2.imshow("left corners", vis_l)

        else:
            print("chessboard not found in {}".format(fn_l))
            continue

        if found_r:
            term_c = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(img_r, corners_r, (5, 5), (-1, -1), term_c)
            vis_r = cv2.cvtColor(img_r, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(vis_r, board_size, corners_r, found_r)
            cv2.imshow("right corners", vis_r)

        else:
            print("chessboard not found in {}".format(fn_r))
            continue

        img_points_l.append(corners_l.reshape(-1, 2))
        img_points_r.append(corners_r.reshape(-1, 2))
        obj_points.append(pattern_points)

        print("corners located")

        key_press = cv2.waitKey(corner_wait_time)

        if key_press == 27 or key_press == 1048603:
            print("Pressed ESC, process stopped")
            break

    cv2.destroyAllWindows()

    rms_l, camera_matrix_l, dist_coefs_l, rvecs_l,\
        tvecs_l = cv2.calibrateCamera(obj_points, img_points_l, (w, h),
                                      None, None)
    print("\nRMS:", rms_l)
    print("Camera matrix:\n", camera_matrix_l)
    print("Distortion coefficients:\n", dist_coefs_l.ravel(), "\n")

    rms_r, camera_matrix_r, dist_coefs_r, rvecs_r,\
        tvecs_r = cv2.calibrateCamera(obj_points, img_points_r, (w, h),
                                      None, None)
    print("\nRMS:", rms_r)
    print("Camera matrix:\n", camera_matrix_r)
    print("Distortion coefficients:\n", dist_coefs_r.ravel(), "\n")

    return camera_matrix_l, camera_matrix_r, dist_coefs_l, dist_coefs_r


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


cameraMatrixL, cameraMatrixR, distCoeffsL, distCoeffsR\
    = calib(imnl, imnr, boardSize, patternPoints)


