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


# imgstowrite: "left", "right", or "stereo"
# Writes <<rangeofimgs>> number of images. "left", "right", or "stereo"
# images based on <<imgstowrite>>
def write_imgs(l_vid, r_vid, imgstowrite, rangeofimgs):
    if not (imgstowrite == "left" or imgstowrite == "right" or
            imgstowrite == "stereo"):
        print("<<imgstowrite>> must be \"left\", \"right\","
              " or \"stereo\"! Writing not started")
        return

    print("Writing {} calibration images... ".format(imgstowrite))
    imgdir = "../data/{}_calib/".format(imgstowrite)
    if not os.path.isdir(imgdir):
        os.mkdir(imgdir)

    for i in rangeofimgs:
        print("    {}: ".format(i), end='')
        l_frame, r_frame, kp = show_frames(l_vid, r_vid)

        if kp == 1048603 or kp == 27:
            print("pressed ESC, image skipped")
            continue

        else:
            num = "{:0>2}".format(i)
            if imgstowrite == "left" or imgstowrite == "right":
                if imgstowrite == "left":
                    frame = l_frame
                else:
                    frame = r_frame
                imwrite(imgdir + imgstowrite + num + ".png", frame)
                print("written")

            else:
                imwrite(imgdir + "left" + num + ".png", l_frame)
                imwrite(imgdir + "right" + num + ".png", r_frame)
                print("written")

    print("... writing finished")
    return


# TODO: Add keywords to get more modability when executing through terminal
if __name__ == '__main__':
    cams = [1, 2]
    leftVid = VideoCapture(cams[0])
    if not leftVid.isOpened():
        print("Can't open vid {}".format(cams[0]))

    rightVid = VideoCapture(cams[1])
    if not rightVid.isOpened():
        print("Can't open vid {}".format(cams[1]))

    print("Left vid size: ")
    print(leftVid.get(CAP_PROP_FRAME_WIDTH))
    print(" x ")
    print(leftVid.get(CAP_PROP_FRAME_HEIGHT))

    numOfImages = 30
    rangeOfImages = range(1, numOfImages + 1)

    write_imgs(leftVid, rightVid, "left", rangeOfImages)
    write_imgs(leftVid, rightVid, "right", rangeOfImages)
    write_imgs(leftVid, rightVid, "stereo", rangeOfImages)

