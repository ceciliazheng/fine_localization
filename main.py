# -*- coding: utf-8 -*-
"""
Created on Thu May  5 15:34:42 2022

@author: Cecilia
"""
from utils import Detection
from utils.image_detection import create_detector

if __name__ == "__main__":
    imagepath = "media/testimage.jpg"
    detection = Detection(imagepath)
    print("width of the april tag detected is: ")
    