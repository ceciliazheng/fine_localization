#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 13 05:11:11 2022

@author: cecilia

Use values calculated from "pose_estimation".
"""

import time
from argparse import ArgumentParser

parser = ArgumentParser(description=__doc__)
parser.add_argument("wpfiles", metavar="WP_FILE", nargs="+")
args = parser.parse_args()
from pymavlink import mavutil, mavwp

from pose_estimation import desired_pose, pose_difference, pose_estimation


def add_header(outf):
        outf.write('''<?xml version="1.0" encoding="UTF-8"?>
                    <gpx
                      version="1.0"
                      creator="pymavlink"
                      xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                      xmlns="http://www.topografix.com/GPX/1/0"
                      xsi:schemaLocation="http://www.topografix.com/GPX/1/0 http://www.topografix.com/GPX/1/0/gpx.xsd">
                    ''')
def add_footer(outf):
        outf.write('''
                    </gpx>
                    ''')
        
def write_wp(imageL, imageR, outfile):
    actualPose, actualOri, distance = pose_estimation(imageL, imageR)
    point2D = []
    desiredPose, desiredOri = desired_pose(point2D)
    diffPose, diffOri = pose_difference(desiredPose, desiredOri, actualPose, actualOri)
    x = diffPose[0]
    y = diffPose[1]
    z = diffPose[2]
    outf = open(outfile, mode = 'w')
    outf.write("QGC WPL 110 \n")
    outf.write('''0\t1\t0\t16\t0.149999999999999994\t0\t0\t0\t"%s"\t"%s"\t"%s"\t 1''') % (x, y, z)
    outf.close()
    return 0

def extract_wp(infilename):
    wp = mavwp.MAVWPLoader()
    wp.load(infilename)
    count = 0
    lat = []
    lon = []
    alt = []
    for i in range(wp.count()):
        w = wp.wp(i)
        t = time.localtime(i)
        if w.frame == 3:
            w.z += wp.wp(0).z
        if w.command == 16:
            lat.append(w.x)
            lon.append(w.y)
            alt.append(w.z)
        count += 1
    return lat, lon, alt
        
def wp_to_gpx(infilename, outfilename):
    '''convert a wp file to a GPX file'''

    wp = mavwp.MAVWPLoader()
    wp.load(infilename)
    outf = open(outfilename, mode='w')

    def process_wp(w, i):
        t = time.localtime(i)
        outf.write('''<wpt lat="%s" lon="%s">
  <ele>%s</ele>
  <cmt>WP %u</cmt>
</wpt>
''' % (w.x, w.y, w.z, i))

    add_header(outf)

    count = 0
    for i in range(wp.count()):
        w = wp.wp(i)
        if w.frame == 3:
            w.z += wp.wp(0).z
        if w.command == 16:
            process_wp(w, i)
        count += 1
    add_footer(outf)
    print("Created %s with %u points" % (outfilename, count))
    