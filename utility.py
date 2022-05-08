# -*- coding: utf-8 -*-
"""
Created on Thu May  5 11:28:50 2022

@author: Cecilia
"""


import argparse

import cv2
import numpy as np


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

def visualize_frame(img, tags):
    color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    for tag in tags:
        # Add bounding rectangle
        for idx in range(len(tag.corners)):
            cv2.line(color_img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)),
                     (0, 255, 0), thickness=3)
        # Add Tag ID text
        cv2.putText(color_img, str(tag.tag_id),
                    org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.8,
                    color=(0, 0, 255),
                    thickness=3)
        # Add Tag Corner
        cv2.circle(color_img, tuple(tag.corners[0].astype(int)), 2, color=(255, 0, 255), thickness=3)
    return color_img