# -*- coding: utf-8 -*-
"""
Created on Thu May  5 12:18:33 2022

@author: Cecilia

This file is part of "ENSNARE fine localization" which is released under GNU 3.0.
See file LICENSE for full license details.

Usage: 
define apriltag coordinates and other necessary parameters as global variables;
call functions as needed in "main.py".
"""

import math
from ctypes.wintypes import tagSIZE

import cv2
from cv2 import DIST_C
import numpy as np

import utils.tag
from utils.image_detection import create_detector, Detection
from utils import Transformation

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
DISTANCE = 100


def load_camera_param():
    '''
    Load intrinsic camera parameters from calibration files.
    Calibration parameters should be stored in "media/calib_results"
    '''
    kl = np.load('media/calib_results/cam_mats_left.npy')
    kr = np.load('media/calib_results/cam_mats_right.npy')
    distl = np.load('media/calib_results/dist_coefs_left.npy')
    distr = np.load('media/calib_results/dist_coefs_right.npy')
    return kl, kr, distl, distr

def camera_pose(tags, Tt, k, dist):
    '''
    Calculate single camera pose from detected apriltags.
    Call Detection in main.py to get tags.
    Input tags in "camera_pose()" for calculation.

    Tt: coordinates of 4 vertices of one tag.
    k: camera matrix
    dist: distorsion coefficient
    '''
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
    '''
    Calculate single camera pose first, 
    then calculate pose of center of two cameras to be used for pose calculation of the drone.
    '''

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

    trans = Transformation(DISTANCE, 0, -50, FOCAL_LENGTH)

    # end goal: calculate drone pose
    dPose = trans.camera_to_drone(cPose[0], cPose[1], cPose[2], rvec[0], rvec[1], rvec[2])
    dOri = rvec
    distance = cPose[0]

    return dPose, dOri, distance

def desired_pose(point2D):
    '''
    Calculate desired pose that is supposed to reached by the drone.
    Movement of drone = desired pose - actual pose
    Pose difference is calculated in "pose_difference()"
    point2D: coordinates of the 4 vertices of a tag in image frame.
    point2D shoul come from manual input. Function called directly in "main.py".
    '''
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
    trans = Transformation(DISTANCE, 0, -50, FOCAL_LENGTH)
    dPose = trans.camera_to_drone(cPose[0], cPose[1], cPose[2], rvec[0], rvec[1], rvec[2])
    dOri = rvec
    
    return dPose, dOri

def pose_difference(desiredPose, desiredOri, actualPose, actualOri):
    diffPose = np.subtract(desiredPose, actualPose)
    diffOri = np.subtracr(desiredOri, actualOri)
    return diffPose, diffOri