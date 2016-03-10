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


