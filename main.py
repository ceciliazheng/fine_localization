# -*- coding: utf-8 -*-
"""
Created on Thu May  5 15:34:42 2022

@author: Cecilia
"""
from apriltag_detection import create_detector


if __name__ == "__main__":
    imagepath = "media/testimage.jpg"
    width = create_detector(imagepath)
    print("width of the april tag detected is: ", width)
    