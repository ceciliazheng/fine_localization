# -*- coding: utf-8 -*-
"""
Created on Thu May  5 11:28:50 2022

@author: Cecilia
"""

import numpy as np
import cv2
import argparse


def parse_images(imagepath):
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", required=True,
                    help = imagepath)
    args = vars(ap.parse_args())
    image = cv2.imread(args["image"], cv2.IMREAD_GRAYSCALE)
    return image


def distance_to_camera(knownWidth, focalLength, perWidth):
    # compute and return the distance from the maker to the camera
    # perWidth: width of the marker in image
	return (knownWidth * focalLength) / perWidth