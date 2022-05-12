# -*- coding: utf-8 -*-
"""
Created on Thu May  5 12:18:33 2022

@author: Cecilia
"""

import math
from ctypes.wintypes import tagSIZE
from time import sleep, time

import cv2
import numpy as np
import numpy.linalg as lin
from image_detection import Detection
from matplotlib import image

import utility
import utils.tag
from utils.image_detection import create_detector

# define KNOWN_WIDTH AND FOCAL_LENGTH
# define world coordinate of april tags
KNOWN_WIDTH = 200
FOCAL_LENGTH = 300
TAG_AX = 0
TAG_AY = 1500
TAG_AZ = 500

TAG_BX = 0
TAG_BY = 1000
TAG_BZ = 500

TAG_CX = 0
TAG_CY = 1000
TAG_CZ = 1000

TAG_DX = 0
TAG_DY = 1500
TAG_DZ = 1000



def camera_to_drone(self, xk, yk, zk, r, p, y):
        '''
        xk: x coordinate of camera frame
        yk: y coordinate of camera frame
        zk: z ccordinate of camera frame
        All with respect to world frame
        '''
        xd = xk - self.dx * math.cos(y)
        yd = yk * math.sin(y)
        zd = self.dy + self.dx * math.sin(p)
        dronePose = [xd, yd, zd, r, p, y]
        return dronePose

def pose_estimation(imagepath):

    detection = Detection(imagepath)
    results = detection.create_detector()
    '''
    read camera intrinsics from calibration
    '''
    kl = np.load('media/calib_results/cam_mats_left.npy')
    kr = np.load('media/calib_results/cam_mats_right.npy')
    distl = np.load('media/calib_results/dist_coefs_left.npy')
    distr = np.load('media/calib_results/dist_coefs_right.npy')
    rot = []
    trans = []

    for r in results:
        (ptA, ptB, ptC, ptD) = r.corners
        (cX, cY) = (r.center[0], r.center[1])
        Tt = np.array([TAG_AX, TAG_AY, TAG_AZ],[TAG_BX, TAG_BY, TAG_BZ], 
                      [TAG_CX, TAG_CY, TAG_CZ], [TAG_DX, TAG_DY, TAG_DZ])
        point2D = np.array([ptA, ptB, ptC, ptD])
        retval, rvec, tvec = cv2.solvePnP(Tt, point2D, kl, distl, useExtrinsicGuess=0, flags=0)

        if len(results) == 1:
            cPose = np.linalg.multi_dot(rvec, Tt) + tvec + Tt
            break
        else:
            rot.append(rvec)
            trans.append(tvec)

    if len(results) > 1:
        rvec = sum(rot) / len(rot)
        tvec = sum(trans) / len(trans)
        cPose = np.linalg.multi_dot(rvec, Tt) + tvec + Tt

    # end goal: calculate drone pose
    dPose = camera_to_drone(cPose)
    dOri = tvec

    return dPose, dOri