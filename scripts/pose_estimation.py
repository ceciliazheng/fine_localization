# -*- coding: utf-8 -*-
"""
Created on Thu May  5 12:18:33 2022

@author: Cecilia
"""

import numpy as np
import cv2
import utility
from apriltag_detection import create_detector


# define KNOWN_WIDTH AND FOCAL_LENGTH
KNOWN_WIDTH = 200
FOCAL_LENGTH = 300
DESIRED_DISTANCE = 500

def pose_estimation(imagepath):
    perWidth = create_detector(imagepath)
    distance = utility.distance_to_camera(KNOWN_WIDTH, FOCAL_LENGTH, perWidth)
    
    # adjust drone position to move to desired distance
    if distance < DESIRED_DISTANCE:
        movement = DESIRED_DISTANCE - distance
    
    if distance > DESIRED_DISTANCE:
        movement = distance - DESIRED_DISTANCE
    