# -*- coding: utf-8 -*-
"""
Created on Thu May  5 12:18:33 2022

@author: Cecilia
"""

from ctypes.wintypes import tagSIZE
import cv2
from matplotlib import image
import numpy as np
import utils.tag
import utility
from utils.apriltag_detection import create_detector
from time import sleep, time
from picamera import PiCamera


# define KNOWN_WIDTH AND FOCAL_LENGTH
KNOWN_WIDTH = 200
FOCAL_LENGTH = 300
DESIRED_DISTANCE = 500


class PoseEstimator:
    def __init__(self, imagepath, tags) -> None:
        self.tags = tags
        self.path = imagepath
        self.images = cv2.imread(imagepath)
        pass

    def estimate_camera_pose(self):
        for t in self.tags:
            print(f"{t.tag_id}")
            print(self.tags.estimate_pose(t.tag_id, t.pose_R, t.pose_t))

        # Visualize The Frame
        cv2.imshow("Visualized Tags", utility.visualize_frame(self.images, self.tags))
        cv2.waitKey(1) & 0xff
        # exit when ESC is pressed
        cv2.destroyAllWindows()  # destroys the window showing image

    def pose_estimation(self):
        perWidth = create_detector(self.imagepath)
        distance = utility.distance_to_camera(KNOWN_WIDTH, FOCAL_LENGTH, perWidth)
        
        # adjust drone position to move to desired distance
        if distance < DESIRED_DISTANCE:
            movement = DESIRED_DISTANCE - distance
        
        if distance > DESIRED_DISTANCE:
            movement = distance - DESIRED_DISTANCE

    def location_adjustment(self):
        # TODO: implementation
        pass
    
    def orientation_adjustment(self):
        # TODO: implementation
        pass