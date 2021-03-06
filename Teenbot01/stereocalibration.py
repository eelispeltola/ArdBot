#!/usr/bin/env python

"""
Stereo camera calibration for two webcams with chess board.
Reads uncalibrated images, calculates calibration matrices and writes
calibrated images.
Project: TeenBot Disparity Map
Author: Eelis Peltola
Started: 10.03.2016
Last modified: 11.03.2016
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


# TODO: Move to own calibration function/modify existing to include stereo
stereo_objpoints = []
stereo_imgpoints_l = []
stereo_imgpoints_r = []
stereo_imgsize = cv2.imread(stereo_imgnames_l[0], 0).shape[:2]
print("Image size:", stereo_imgsize)

cv2.namedWindow("left corners")
cv2.namedWindow("right corners")

print("Locating chessboard corners in images \n    {} and \n    {}"
      .format(stereo_imgmask_l, stereo_imgmask_r))

for fnL, fnR in zip(stereo_imgnames_l, stereo_imgnames_r):
    print("Reading {} and {}... ".format(fnL.split('/')[-1],
                                         fnR.split('/')[-1]), end='')
    imgL = cv2.imread(fnL, 0)
    imgR = cv2.imread(fnR, 0)

    if imgL is None:
        print("failed to load ", fnL)
        continue

    if imgR is None:
        print("failed to load ", fnR)
        continue

    if imgL.shape[:2] != stereo_imgsize or imgR.shape[:2] != stereo_imgsize:
        print("dimensions of {} or {} do not match".format(fnL, fnR))
        continue

#    h, w = imgL.shape[:2]
    foundL, cornersL = cv2.findChessboardCorners(imgL, boardSize,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH |
                                                 cv2.CALIB_CB_NORMALIZE_IMAGE)
    foundR, cornersR = cv2.findChessboardCorners(imgR, boardSize,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH |
                                                 cv2.CALIB_CB_NORMALIZE_IMAGE)

    if foundL:
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
        cv2.cornerSubPix(imgL, cornersL, (5, 5), (-1, -1), term)
        visL = cv2.cvtColor(imgL, cv2.COLOR_GRAY2BGR)
        cv2.drawChessboardCorners(visL, boardSize, cornersL, foundL)
        cv2.imshow("left corners", visL)

    else:
        print("chessboard not found in {}".format(fnL))
        continue

    if foundR:
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
        cv2.cornerSubPix(imgR, cornersR, (5, 5), (-1, -1), term)
        visR = cv2.cvtColor(imgR, cv2.COLOR_GRAY2BGR)
        cv2.drawChessboardCorners(visR, boardSize, cornersR, foundR)
        cv2.imshow("right corners", visR)

    else:
        print("chessboard not found in {}".format(fnR))
        continue

    stereo_imgpoints_l.append(cornersL.reshape(-1, 2))
    stereo_imgpoints_r.append(cornersR.reshape(-1, 2))
    stereo_objpoints.append(patternPoints)

    print("corners located")

    keyPress = cv2.waitKey(corner_wait_time)

    if keyPress == 27 or keyPress == 1048603:
        print("Pressed ESC, process stopped")
        break

cv2.destroyAllWindows()

print("\nRunning stereo calibration... ", end='')

rms, cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, R, T, E, F \
    = cv2.stereoCalibrate(objectPoints=stereo_objpoints,
                          imagePoints1=stereo_imgpoints_l,
                          imagePoints2=stereo_imgpoints_r,
                          cameraMatrix1=cameraMatrixL,
                          distCoeffs1=distCoeffsL,
                          cameraMatrix2=cameraMatrixR,
                          distCoeffs2=distCoeffsR,
                          imageSize=stereo_imgsize,
                          R=None, T=None, E=None, F=None,
                          flags=(cv2.CALIB_FIX_ASPECT_RATIO +
                                 cv2.CALIB_FIX_INTRINSIC),
                          criteria=(cv2.TERM_CRITERIA_COUNT +
                                    cv2.TERM_CRITERIA_EPS, 300, 1e-6))

print("calibrated with RMS error =", rms)
print("\nCamera matrices (L and R respectively):\n", cameraMatrixL,
      "\n \n", cameraMatrixR)
print("\nDistortion coefficients:\n", distCoeffsL.ravel(), "\n \n",
      distCoeffsR.ravel(), "\n")
print("Rotational matrix between camera 1 and camera 2 \n", R, "\n")
print("Translational matrix between camera 1 and camera 2 \n", T, "\n")

print("Writing intrinsics to /data/intrinsics.yml... ", end='')
with open("../data/instrinsics.yml", 'w') as intrStream:
    yaml.dump_all(["CM1", cameraMatrixL, "D1", distCoeffsL, "CM2",
                   cameraMatrixR, "D2", distCoeffsR], intrStream)
print("intrinsics written")


print("Writing extrinsics to /data/extrinsics.yml... ", end='')
RL, RR, PL, PR, Q, \
    roiL, roiR = cv2.stereoRectify(cameraMatrix1=cameraMatrixL,
                                   distCoeffs1=distCoeffsL,
                                   cameraMatrix2=cameraMatrixR,
                                   distCoeffs2=distCoeffsR,
                                   imageSize=stereo_imgsize,
                                   R=R, T=T, R1=None, R2=None,
                                   P1=None, P2=None, Q=None,
                                   alpha=0,
                                   newImageSize=stereo_imgsize)
with open("../data/extrinsics.yml", 'w') as extrStream:
    yaml.dump_all(["R1", RL, "R2", RR, "P1", PL, "P2", PR, "Q", Q], extrStream)
print("extrinsics written")
print("R1\n", RL, "\nR2\n", RR, "\nP1\n", PL, "\nP2\n", PR, "\nQ\n", Q,
      "\nroiL\n", roiL, "\nroiR\n", roiR)


# TODO: Move to depth-mapping module? Or own debug function

print("Undistorting images... ", end='')

# Invert image size to width*height
distImgSize = (stereo_imgsize[1], stereo_imgsize[0])

mapLx, mapLy = cv2.initUndistortRectifyMap(cameraMatrixL, distCoeffsL, RL,
                                           PL, distImgSize, cv2.CV_16SC2, None,
                                           None)

mapRx, mapRy = cv2.initUndistortRectifyMap(cameraMatrixR, distCoeffsR, RR,
                                           PR, distImgSize, cv2.CV_16SC2, None,
                                           None)

# Show remapped versions of calibration images
keyPressFlag = False
cv2.namedWindow("Remapped left")
cv2.namedWindow("Remapped right")
for imL, imR in zip(stereo_imgnames_l, stereo_imgnames_r):
    imgL = cv2.imread(imL)
    remappedImgL = cv2.remap(src=imgL, map1=mapLx, map2=mapLy,
                             interpolation=cv2.INTER_LINEAR)
    cv2.imshow("Remapped left", remappedImgL)

    imgR = cv2.imread(imR)
    remappedImgR = cv2.remap(src=imgR, map1=mapRx, map2=mapRy,
                             interpolation=cv2.INTER_LINEAR)
    cv2.imshow("Remapped right", remappedImgR)

    keyPress = cv2.waitKey(1500)
    if keyPress == 27 or keyPress == 1048603:
        print("pressed ESC, process stopped")
        keyPressFlag = True
        break

if not keyPressFlag:
    print("undistort complete")



