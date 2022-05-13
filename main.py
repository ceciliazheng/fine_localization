# -*- coding: utf-8 -*-
"""
Created on Thu May  5 15:34:42 2022

@author: Cecilia
"""
from pose_estimation import desired_pose, pose_difference, pose_estimation
from utils import Detection
from utils.image_detection import create_detector

ALLOWED_ERROR = 1

if __name__ == "__main__":
    imagepathL = "media/testimage_left.jpg"
    imagepathR = "media/testimage_right.jpg"
    # input april tag coordinates in image
    point2D = []

    detection = Detection(imagepathL)
    print("width of the april tag detected is: ")
    actualPose, actualOri, actualDistance = pose_estimation(imagepathL, imagepathR)
    desiredPose, desiredOri = desired_pose(point2D)
    diffPose, diffOri = pose_difference(desiredPose, desiredOri, actualPose, actualOri)


    