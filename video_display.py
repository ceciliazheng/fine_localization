# -*- coding: utf-8 -*-
from datetime import date, datetime
import time
import cv2
import numpy as np
import json
from arducam_camera import MyCamera
from stereovision.calibration import StereoCalibrator
from stereovision.calibration import StereoCalibration



def video_display():

    try:
        camera_params = json.load(open("camera_params.txt", "r"))
    except Exception as e:
        print(e)
        print("Camera parameters don't exist.")
        exit(-1)

    # Initialize the camera
    camera = MyCamera()

    # Camera settimgs
    cam_width = camera_params['width']    # Cam sensor width settings
    cam_height = camera_params['height']  # Cam sensor height settings
    print ("Used camera resolution: "+str(cam_width)+" x "+str(cam_height))

    print("Open camera...")
    camera.open_camera(camera_params['device'], cam_width, cam_height)
    fmt = camera.get_framesize()
    print("Current resolution: {}x{}".format(fmt[0], fmt[1]))

    scale = camera_params['scale']
    # Buffer for captured image settings
    img_width = int(camera_params['width'] * scale)
    img_height = int(camera_params['height'] * scale)
    print ("Scaled image resolution: "+str(img_width)+" x "+str(img_height))

    # Implement calibration data
    print('Read calibration data and rectifying stereo pair...')
    calibration = StereoCalibration(input_folder='media/calib_result')

    # Initialize interface windows
    cv2.namedWindow("Image")
    cv2.moveWindow("Image", 50,100)
    cv2.namedWindow("left")
    cv2.moveWindow("left", 450,100)
    cv2.namedWindow("right")
    cv2.moveWindow("right", 850,100)

    while True:
        frame = camera.get_frame()
        t1 = datetime.now
        # pair_img = frame
        pair_img = cv2.cvtColor (frame, cv2.COLOR_BGR2GRAY)
        imgLeft = pair_img [0:img_height,0:int(img_width/2)] #Y+H and X+W
        imgRight = pair_img [0:img_height,int(img_width/2):img_width] #Y+H and X+W
        rectified_pair = calibration.rectify((imgLeft, imgRight))
        cv2.imshow("left", imgLeft)
        cv2.imshow("right", imgRight)  
        t2 = datetime.now()
        print ("DM build time: " + str(t2-t1))
    
    camera.close_camera()




    '''
    
    # show the frame
    cv2.imshow("left", imgLeft)
    cv2.imshow("right", imgRight)    

    t2 = datetime.now()
    print ("DM build time: " + str(t2-t1))

    camera.close_camera()
    '''
