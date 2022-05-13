# -*- coding: utf-8 -*-
"""
Created on Thu May  5 12:18:33 2022

@author: Cecilia
"""

import math
from ctypes.wintypes import tagSIZE

import cv2
import numpy as np
from image_detection import Detection

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
RES = [1600, 1300]


def load_camera_param():
    kl = np.load('media/calib_results/cam_mats_left.npy')
    kr = np.load('media/calib_results/cam_mats_right.npy')
    distl = np.load('media/calib_results/dist_coefs_left.npy')
    distr = np.load('media/calib_results/dist_coefs_right.npy')
    return kl, kr, distl, distr

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

def camera_pose(tags, Tt, k, dist):
    rot = []
    trans = []
    for t in tags:
        (ptA, ptB, ptC, ptD) = t.corners
        (cX, cY) = (t.center[0], t.center[1])
        point2D = np.array([ptA, ptB, ptC, ptD])
        retval, rvec, tvec = cv2.solvePnP(Tt, point2D, k, dist, useExtrinsicGuess=0, flags=0)

        if len(tags) == 1:
            cPose = np.linalg.multi_dot(rvec, Tt) + tvec + Tt
            break
        else:
            rot.append(rvec)
            trans.append(tvec)

    if len(tags) > 1:
        rvec = sum(rot) / len(rot)
        tvec = sum(trans) / len(trans)
        cPose = np.linalg.multi_dot(rvec, Tt) + tvec + Tt

    return cPose, rvec

def pose_estimation(leftpath, rightpath):

    detectionL = Detection(leftpath)
    detectionR = Detection(rightpath)
    resultsL = detectionL.create_detector()
    resultsR = detectionR.create_detector()
    '''
    read camera intrinsics from calibration
    '''
    kl, kr, distl, distr = load_camera_param()
    Tt = np.array([TAG_AX, TAG_AY, TAG_AZ],[TAG_BX, TAG_BY, TAG_BZ], 
                  [TAG_CX, TAG_CY, TAG_CZ], [TAG_DX, TAG_DY, TAG_DZ])

    cPoseL, rvecl = camera_pose(resultsL, Tt, kl, distl)
    cPoseR, rvecr = camera_pose(resultsR, Tt, kr, distr)
    cPose = sum(cPoseL, cPoseR) / 2
    rvec = sum(rvecl, rvecr) / 2

    # end goal: calculate drone pose
    dPose = camera_to_drone(cPose)
    dOri = rvec
    distance = cPose[0]

    return dPose, dOri, distance

def desired_pose(point2D):
    kl, kr, distl, distr = load_camera_param()
    # point2D = np.array([ptA, ptB, ptC, ptD])
    distance = 1000
    x = distance
    y = 0
    z = 1000
    Tt = np.array([TAG_AX, TAG_AY, TAG_AZ],[TAG_BX, TAG_BY, TAG_BZ], 
                  [TAG_CX, TAG_CY, TAG_CZ], [TAG_DX, TAG_DY, TAG_DZ])
    retval, rvec, tvec = cv2.solvePnP(Tt, point2D, kl, distl, useExtrinsicGuess=0, flags=0)
    cPose = np.linalg.multi_dot(rvec, Tt) + tvec + Tt
    dPose = camera_to_drone(cPose)
    dOri = rvec
    
    return dPose, dOri

def pose_difference(desiredPose, desiredOri, actualPose, actualOri):
    diffPose = np.subtract(desiredPose, actualPose)
    diffOri = np.subtracr(desiredOri, actualOri)
    return diffPose, diffOri