# -*- coding: utf-8 -*-
import json
import time
from datetime import date, datetime

import cv2
import numpy as np
import pupil_apriltags as apriltag
from scipy.spatial import distance
from stereovision.calibration import StereoCalibration, StereoCalibrator

from arducam_camera import MyCamera


class Video:
    def __init__(self) -> None:
        camera = MyCamera()
        self.camera = camera
        self.frame = None
        self.left = None
        self.right = None
        self.tags = None
        self.leftVertices = []
        self.leftCenterPts = []
        self.leftPerWidth = []
        self.rightVertices = []
        self.rightCenterPts = []
        self.rightPerWidth = []
        pass

    def video_display(self):
        try:
            camera_params = json.load(open("camera_params.txt", "r"))
        except Exception as e:
            print(e)
            print("Camera parameters don't exist.")
            exit(-1)

        # Initialize the camera
        camera = self.camera

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

        frame = camera.get_frame()
        self.frame = frame
        t1 = datetime.now
        # pair_img = frame
        pair_img = cv2.cvtColor (frame, cv2.COLOR_BGR2GRAY)
        imgLeft = pair_img [0:img_height,0:int(img_width/2)] #Y+H and X+W
        imgRight = pair_img [0:img_height,int(img_width/2):img_width] #Y+H and X+W
        rectified_pair = calibration.rectify((imgLeft, imgRight))
        cv2.imshow("left", imgLeft)
        cv2.imshow("right", imgRight) 
        self.left = imgLeft
        self.right = imgRight 
        t2 = datetime.now()
        print ("DM build time: " + str(t2-t1))

        def end_display(self):
            self.camera.close_camera()
        
        def detect(self):
            detector = apriltag.Detector(families='tag36h11')
            leftResults = detector.detect(self.left)
            rightResults = detector.detect(self.right)
            print("[INFO] In left image, {} total AprilTags detected".format(len(leftResults)))
            print("[INFO] In right image, {} total AprilTags detected".format(len(rightResults)))
            leftVertices = []
            leftCenterPts = []
            leftPerWidth = []
            rightVertices = []
            rightCenterPts = []
            rightPerWidth = []

            for r in leftResults:
                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                leftVertices.append([ptA, ptB, ptC, ptD])

                width = distance.euclidean(ptA, ptB)
                # width unit: mm
                leftPerWidth.append(width)
                
                # draw the bounding box of the AprilTag detection
                cv2.line(self.image, ptA, ptB, (0, 255, 0), 2)
                cv2.line(self.image, ptB, ptC, (0, 255, 0), 2)
                cv2.line(self.image, ptC, ptD, (0, 255, 0), 2)
                cv2.line(self.image, ptD, ptA, (0, 255, 0), 2)
                
                # draw the center (x, y)-coordinates of the AprilTag
                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                cv2.circle(self.image, (cX, cY), 5, (0, 0, 255), -1)
                center = (cX, cY)
                leftCenterPts.append(center)

            for r in rightResults:
                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                rightVertices.append([ptA, ptB, ptC, ptD])

                width = distance.euclidean(ptA, ptB)
                # width unit: mm
                rightPerWidth.append(width)
                
                # draw the bounding box of the AprilTag detection
                cv2.line(self.image, ptA, ptB, (0, 255, 0), 2)
                cv2.line(self.image, ptB, ptC, (0, 255, 0), 2)
                cv2.line(self.image, ptC, ptD, (0, 255, 0), 2)
                cv2.line(self.image, ptD, ptA, (0, 255, 0), 2)
                
                # draw the center (x, y)-coordinates of the AprilTag
                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                cv2.circle(self.image, (cX, cY), 5, (0, 0, 255), -1)
                center = (cX, cY)
                rightCenterPts.append(center)

            self.leftCenterPts.append(leftCenterPts)
            self.leftVertices.append(leftVertices)
            self.leftPerWidth.append(leftPerWidth)
            self.rightCenterPts.append(rightCenterPts)
            self.rightVertices.append(rightVertices)
            self.rightPerWidth.append(rightPerWidth)
