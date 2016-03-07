#!/usr/usr/bin/env python

"""
Write camera calibration images for desired camera to data folder.
First left, then right, then both at the same time.
Project: TeenBot Disparity Map
Author: Eelis Peltola
Started: 07.03.2016
Last modified: 07.03.2016
"""

from cv2 import *
import os


# Show video feed from cameras <<l_vid>> and <<r_vid>>
def show_frames(l_vid, r_vid):
    keypress = waitKey(1)
    vid_read_l, left_frame = leftVid.read()
    vid_read_r, right_frame = rightVid.read()
    while keypress == -1:
        vid_read_l, left_frame = leftVid.read()
        vid_read_r, right_frame = rightVid.read()
        if not vid_read_l:
            print("Couldn't grab frame from left camera")
            break

        if not vid_read_r:
            print("Couldn't grab frame from right camera")
            break

        imshow("Left Frame", left_frame)
        imshow("Right Frame", right_frame)
        keypress = waitKey(25)
    return left_frame, right_frame, keypress
