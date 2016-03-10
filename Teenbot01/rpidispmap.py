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


# Read videos and remap read frames based on undistort maps
def remap_images(l_video, r_video, leftmap_x, leftmap_y,
                 rigthmap_x, rightmap_y):
    vidreadl, leftframe = l_video.read()
    vidreadr, rightframe = r_video.read()
    if not vidreadl:
        print("Couldn't grab frame from left camera")
        return

    if not vidreadr:
        print("Couldn't grab frame from right camera")
        return

    remappedimgl = remap(src=leftframe, map1=leftmap_x, map2=leftmap_y,
                         interpolation=INTER_LINEAR)
    imshow("Remapped left", remappedimgl)

    remappedimgr = remap(src=rightframe, map1=rigthmap_x, map2=rightmap_y,
                         interpolation=INTER_LINEAR)
    imshow("Remapped right", remappedimgr)

    return remappedimgl, remappedimgr, leftframe, rightframe


# Set video feeds to <<video_size>> or closest equivalent
# (set video frames to 144p for performance)
def set_video_sizes(left_vid, right_vid, video_size):
    print("Unmodified left vid size: ")
    print(left_vid.get(CAP_PROP_FRAME_WIDTH))
    print(" x ")
    print(left_vid.get(CAP_PROP_FRAME_HEIGHT))

    print("Unmodified right vid size: ")
    print(right_vid.get(CAP_PROP_FRAME_WIDTH))
    print(" x ")
    print(right_vid.get(CAP_PROP_FRAME_HEIGHT))

    width = video_size[0]
    height = video_size[1]
    left_vid.set(CAP_PROP_FRAME_WIDTH, width)
    left_vid.set(CAP_PROP_FRAME_HEIGHT, height)
    right_vid.set(CAP_PROP_FRAME_WIDTH, width)
    right_vid.set(CAP_PROP_FRAME_HEIGHT, height)

    print("Modified video size: ")
    print(right_vid.get(CAP_PROP_FRAME_WIDTH))
    print(" x ")
    print(right_vid.get(CAP_PROP_FRAME_HEIGHT))


if __name__ == '__main__':
    leftVid = VideoCapture(1)
    if not leftVid.isOpened():
        print("Can't open vid 0")

    rightVid = VideoCapture(2)
    if not rightVid.isOpened():
        print("Can't open vid 1")

    # Desired image size as (width*height)
    # (set video frames to 144p for performance)
    imgSize = (640, 480)

    set_video_sizes(leftVid, rightVid, imgSize)

    namedWindow('tracks', 1)
    createTrackbar('Disparities', 'tracks', 1, 15, nothing)
    createTrackbar('minDisparity', 'tracks', 0, 3, nothing)
    createTrackbar('minDispSign', 'tracks', 0, 1, nothing)
    createTrackbar('blockSize', 'tracks', 3, 50, nothing)
    createTrackbar('windowSize', 'tracks', 3, 50, nothing)
    createTrackbar('disp12MaxDiff', 'tracks', 10, 200, nothing)
    createTrackbar('preFilterCap', 'tracks', 5, 200, nothing)
    createTrackbar('uniqueness', 'tracks', 0, 10, nothing)
    createTrackbar('SpeckleWindowSize', 'tracks', 50, 150, nothing)
    createTrackbar('SpeckleRange', 'tracks', 2, 50, nothing)

    mapLx, mapLy, mapRx, mapRy = calculate_maps(imgSize)

    namedWindow("Remapped left")
    namedWindow("Remapped right")

    print("Calculating disparity map... ", end='')

    while True:

        remappedImgL, remappedImgR, leftFrame, rightFrame\
            = remap_images(leftVid, rightVid, mapLx, mapLy, mapRx, mapRy)

 	# StereoSGBM values from trackbar for testing
        min_disp = getTrackbarPos('minDisparity', 'tracks') * \
            (1 - 2*getTrackbarPos('minDispSign', 'tracks'))*16
        num_disp = getTrackbarPos('Disparities', 'tracks')*16 + 16
        block_sz = getTrackbarPos('blockSize', 'tracks')
        window_sz = getTrackbarPos('windowSize', 'tracks')
        disp12_maxdiff = getTrackbarPos('disp12MaxDiff', 'tracks')
        pre_filter_cap = getTrackbarPos('preFilterCap', 'tracks')
        uniqueness = getTrackbarPos('uniqueness', 'tracks')
        speckle_window_sz = getTrackbarPos('SpeckleWindowSize', 'tracks')
        speckle_range = getTrackbarPos('SpeckleRange', 'tracks')
        mode = STEREO_SGBM_MODE_HH

        stereo = StereoSGBM_create(minDisparity=min_disp,
                                   numDisparities=num_disp,
                                   blockSize=block_sz,
                                   P1=8*3*window_sz**2,
                                   P2=32*3*window_sz**2,
                                   disp12MaxDiff=disp12_maxdiff,
                                   preFilterCap=pre_filter_cap,
                                   uniquenessRatio=uniqueness,
                                   speckleWindowSize=speckle_window_sz,
                                   speckleRange=speckle_range,
                                   mode=mode
                                   )

        # Alternative way to form stereo image:
        '''
        stereo = StereoBM_create(32, 31)
        stereo.setDisp12MaxDiff(1)
        stereo.setSpeckleRange(10)
        stereo.setSpeckleWindowSize(9)
        stereo.setUniquenessRatio(0)
        stereo.setTextureThreshold(50)
        stereo.setMinDisparity(-16)
        stereo.setPreFilterCap(61)
        stereo.setPreFilterSize(5)
        '''

        g1 = cvtColor(remappedImgL, COLOR_BGR2GRAY)
        g2 = cvtColor(remappedImgR, COLOR_BGR2GRAY)

        disparity = stereo.compute(g1, g2)
        disparity8 = np.zeros((imgSize[0], imgSize[1], 3), np.uint8)
        disparity8 = normalize(disparity, disparity8, 0, 255, NORM_MINMAX,
                               CV_8U)

        imshow("leftFrame", leftFrame)
        imshow("rightFrame", rightFrame)
        imshow("disparity", disparity8)

        keyPress = waitKey(20)
        if keyPress == 27 or keyPress == 1048603:
            print("Pressed ESC key, terminating")
            break

    destroyAllWindows()
    leftVid.release()
    rightVid.release()


