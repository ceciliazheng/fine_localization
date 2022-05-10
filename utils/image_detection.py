# -*- coding: utf-8 -*-
"""
Created on Thu May  5 10:27:46 2022

@author: Cecilia

requirements.txt
Creating a detector and realize Apriltag detection.

"""

import cv2
import numpy as np

from pupil_apriltags import Detector
from scipy.spatial import distance

import utility


class Detection:
	def __init__(self, imagepath) -> None:
		self.path = imagepath
		self.image = None
		self.detector = None
		self.tags = None
		self.vertcies = []
		self.centerPts = []
		self.perWidth = []
		self.pose = []
	
	def create_detector(self):
		self.image = utility.parse_images(self.path)
		apriltagFamily = "NameOfFamily"
		detector = Detector(families='tag36h11')
		results = detector.detect(self.image)
		self.tags = results
		print("[INFO] {} total AprilTags detected".format(len(results)))
		
	def get_coordinates(self):
		# loop over the AprilTag detection results
		vertices = []
		centerPts = []
		perWidth = []

		for r in self.tags:
			# extract the bounding box (x, y)-coordinates for the AprilTag
			# and convert each of the (x, y)-coordinate pairs to integers
			(ptA, ptB, ptC, ptD) = r.corners
			ptB = (int(ptB[0]), int(ptB[1]))
			ptC = (int(ptC[0]), int(ptC[1]))
			ptD = (int(ptD[0]), int(ptD[1]))
			ptA = (int(ptA[0]), int(ptA[1]))
			vertices.append([ptA, ptB, ptC, ptD])

			width = distance.euclidean(ptA, ptB)
			# width unit: mm
			perWidth.append(width)
			
			# draw the bounding box of the AprilTag detection
			cv2.line(self.image, ptA, ptB, (0, 255, 0), 2)
			cv2.line(self.image, ptB, ptC, (0, 255, 0), 2)
			cv2.line(self.image, ptC, ptD, (0, 255, 0), 2)
			cv2.line(self.image, ptD, ptA, (0, 255, 0), 2)
			
			# draw the center (x, y)-coordinates of the AprilTag
			(cX, cY) = (int(r.center[0]), int(r.center[1]))
			cv2.circle(self.image, (cX, cY), 5, (0, 0, 255), -1)
			center = (cX, cY)
			centerPts.append(center)
		
		# draw the tag family on the image
		tagFamily = r.tag_family.decode("utf-8")
		cv2.putText(self.image, tagFamily, (ptA[0], ptA[1] - 15),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
		print("[INFO] tag family: {}".format(tagFamily))

		'''
		To access values - example:
		tf = result[0].tag_family
		cx = result[0].center[0]
		'''
		print("Coordinates of the center points: ", centerPts)

		# show the output image after AprilTag detection
		cv2.imshow("Image", self.image)
		cv2.waitKey(5000)
		cv2.destroyAllWindows()

		self.centerPts.append(centerPts)
		self.vertcies.append(vertices)
		self.perWidth.append(perWidth)
	
	def estimate_camera_pose(self):
		pose = []
		for t in self.tags:
			print(f"{t.tag_id}")
			pose.append(t.estimate_pose(t.tag_id, t.pose_R, t.pose_t))
			print(t.estimate_pose(t.tag_id, t.pose_R, t.pose_t))

        # Visualize The Frame
		cv2.imshow("Visualized Tags", utility.visualize_frame(self.images, self.tags))
		cv2.waitKey(1) & 0xff
        # exit when ESC is pressed
		cv2.destroyAllWindows()  # destroys the window showing image
		self.pose = pose

	def estimate_distance(self):
		
		