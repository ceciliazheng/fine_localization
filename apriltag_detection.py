# -*- coding: utf-8 -*-
"""
Created on Thu May  5 10:27:46 2022

@author: Cecilia

requirements.txt
Creating a detector and realize Apriltag detection.

"""

import cv2
import numpy as np
import utility
from scipy.spatial import distance
from pupil_apriltags import Detector


def create_detector(imagepath):
	# Define which family of apriltag we are using - 'Tag36h11'
	# More options can also be added to 'options'
	image = utility.parse_images(imagepath)
	apriltagFamily = "NameOfFamily"
	# Tuning detector parameters
	# options = apriltag.DetectorOptions(families=apriltagFamily)
	detector = Detector(families='tag36h11')
	results = detector.detect(image)
	print("[INFO] {} total AprilTags detected".format(len(results)))
	perWidth = []

	# loop over the AprilTag detection results
	for r in results:
		# extract the bounding box (x, y)-coordinates for the AprilTag
		# and convert each of the (x, y)-coordinate pairs to integers
		(ptA, ptB, ptC, ptD) = r.corners
		ptB = (int(ptB[0]), int(ptB[1]))
		ptC = (int(ptC[0]), int(ptC[1]))
		ptD = (int(ptD[0]), int(ptD[1]))
		ptA = (int(ptA[0]), int(ptA[1]))

		width = distance.euclidean(ptA, ptB)
		# width unit: mm
		perWidth.append(width)
		
		# draw the bounding box of the AprilTag detection
		cv2.line(image, ptA, ptB, (0, 255, 0), 2)
		cv2.line(image, ptB, ptC, (0, 255, 0), 2)
		cv2.line(image, ptC, ptD, (0, 255, 0), 2)
		cv2.line(image, ptD, ptA, (0, 255, 0), 2)
		
		# draw the center (x, y)-coordinates of the AprilTag
		(cX, cY) = (int(r.center[0]), int(r.center[1]))
		cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
		
		# draw the tag family on the image
		tagFamily = r.tag_family.decode("utf-8")
		cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
		print("[INFO] tag family: {}".format(tagFamily))

	'''
	To access values - example:
	tf = result[0].tag_family
	cx = result[0].center[0]
	'''

	# show the output image after AprilTag detection
	cv2.imshow("Image", image)
	cv2.waitKey(5000)
	cv2.destroyAllWindows()

	return perWidth

#TODO: distance estimation
