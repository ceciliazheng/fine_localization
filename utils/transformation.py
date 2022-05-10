# -*- coding: utf-8 -*-
"""
Created on Tue May  10 19:03:45 2022

@author: Cecilia
"""
import math
import utility
from utils.pose_estimation import FOCAL_LENGTH, KNOWN_WIDTH


class Transformation(object):
    def __init__(self, length, width, height, f) -> None:
        self.dx = length
        self.dy = width
        self.dz = height
        self.focal = f
        self.camera_trans = []
        pass

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

    def camera_translation(self, res, d, xt, yt, x, y, z):
        '''
        d: distance of camera to apriltag
        xt: x coordinate of tag center in image
        yt: y coordinate of tag center in image
        res: resolution of camera
        x, y, z: tag coordinate in world frame
        '''
        dx = res[0]/2 - xt
        dy = res[1]/2 - yt
        yk = y + (dx * d) / self.focal
        zk = z + (dy * d) / self.focal
        xk = d
        translation = [xk, yk, zk]
        self.camera_trans = translation
        '''
        INCORRECT
        '''
        return translation

    def camera_rotation(self):
        pass

    def distance_to_camera(self, perWidth):
        KNOWN_WIDTH = 20
        FOCAL_LENGTH = 300
        distance = utility.distance_to_camera(KNOWN_WIDTH, FOCAL_LENGTH, perWidth)
        # distance of edge of the tag to camera
        return distance

