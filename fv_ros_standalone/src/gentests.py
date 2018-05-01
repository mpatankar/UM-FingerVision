#!/usr/bin/python
#\file    gentests.py
#\date    Apr.4, 2018
#
#coordinates:
#
#                      +z ^ (finger tip)
#                         |
#+y(down into finger tip) o--> +x (finger right side)
import roslib#; roslib.load_manifest('lfd_vision')
import rospy
import fv_standalone.msg
import math
import numpy as np
import numpy.linalg as la
import json
import argparse

from math import cos, sin
from fv_standalone.srv import *

import sys

calib_dir = '../calib/'

g = 9.81

gen_samples = True
num_samples = 10 #collect 10 samples
sample_count = 0 

frame_height = 0
frame_width = 0
frame_data = []
calib_data = {} #dictionary representing calibation data for 1 configuration (force, thetax, thetaz)
move_data = {} #dictionary representing one Blob_Moves sample point
blob_move = {} #dictionary representing one Blob_Move object
#frame data: Pox, Poy, So, Dpx, Dpy, DS

def GetForces(mass, thetax, thetaz):
	weight = mass * g / 1000.0
	fxy = weight * cos(np.deg2rad(thetaz))
	fxz = weight * cos(np.deg2rad(thetax))

	fx = abs(fxy) * sin(np.deg2rad(thetax))
	fz = abs(fxz) * sin(np.deg2rad(thetaz))
	fy = abs(abs(fxy) * cos(np.deg2rad(thetax)))

	return (fx, fy, fz)

"""
def BlobMoves(msg):
	global sample_count, num_samples, frame_width, frame_height
	if sample_count < num_samples:
		frame_width = msg.width;
		frame_height = msg.height;

		move_data['id'] = sample_count
		move_data['BlobMoves'] = []
	
		for mv in msg.data:
			move_data['BlobMoves'] += [[mv.Pox, mv.Poy, mv.So, mv.DPx, mv.DPy, mv.DS]]

		calib_data['MoveData'] += [dict(move_data)]

		sample_count+=1
	
		#print calib_data

"""

def get_blob_moves(file_name):
	rospy.wait_for_service('fv_standalone/collect_data')
	try:
		collect_data = rospy.ServiceProxy('fv_standalone/collect_data', CollectData)
		response = collect_data(file_name)
		return response
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__=='__main__':
  #rospy.init_node('fv_test_force')
  #Subscribing blob movement topic (marker tracking)

  #sub_bm = rospy.Subscriber("fv_standalone/blob_moves",
  #                         fv_standalone.msg.BlobMoves, BlobMoves)

  parser = argparse.ArgumentParser()
  parser.add_argument("--mode_specific", "-s", help="test step mode for entering specific config arguments: [mass (g), thetax (rad), thetaz (rad), torque (N-m) - ccw = positive]", action="store", dest="specific", nargs=4, type=float)
  parser.add_argument("--mode_json", '-j', help="json file containing list of tests: [mass (g), thetax (rad), thetaz (rad), torque (N-m) - ccw = positive]", action="store", dest="jsonfilename", nargs=1, type=str)
  args = parser.parse_args()

  calib_list = []

  if args.specific:
	calib_list += [[args.specific[0], args.specific[1], args.specific[2], args.specific[3]]]
  elif args.jsonfilename:
	calib_list = json.load(open(args.jsonfilename[0]))['tests']

  print "calibration list: "
  print calib_list

#  n = 0
  
#  calib_data['MoveData'] = []
#  raw_input("working on config: (mass:{:.4f}, thetax:{:.4f}, thetaz:{:.4f}, torque:{:.4f}) - Press any key to collect data".format(calib_list[n][0], calib_list[n][1], calib_list[n][2], calib_list[n][3]))

  for config in calib_list:

	raw_input("working on config: (mass:{:.4f}, thetax:{:.4f}, thetaz:{:.4f}, torque:{:.4f}) - Press any key to collect data".format(config[0], config[1], config[2], config[3]))


	calib_data['mass'] = config[0]
	calib_data['thetax'] = config[1]
	calib_data['thetaz'] = config[2]
	calib_data['torque'] = config[3]
	expected_f = GetForces(calib_data['mass'], calib_data['thetax'], calib_data['thetaz'])

	calib_data['fx'] = expected_f[0]
	calib_data['fy'] = expected_f[1]
	calib_data['fz'] = expected_f[2]

	calib_data['MoveData'] = []

	for n in range(10):
		
		fname = "{:.2f}_{:.2f}_{:.2f}_{:.2f}_{}.jpg".format(config[0], config[1], config[2], config[3], n)
		response = get_blob_moves(fname)

		if n == 0:
			calib_data['frame_width'] = response.width
			calib_data['frame_height'] = response.height
		
		move_data['id'] = n
		move_data['BlobMoves'] = []
	
		for mv in response.data:
			move_data['BlobMoves'] += [[mv.Pox, mv.Poy, mv.So, mv.DPx, mv.DPy, mv.DS]]

		calib_data['MoveData'] += [dict(move_data)]
		
		print response

	fname = calib_dir+"calib_{:.2f}_{:.2f}_{:.2f}_{:.2f}.json".format(calib_data['mass'], calib_data['thetax'], calib_data['thetaz'], calib_data['torque'])

	with open(fname, 'w') as fp:
		json.dump(calib_data, fp)	

  sys.exit()
"""
  while not rospy.core.is_shutdown():
  	rospy.rostime.wallsleep(0.5)

	if sample_count >= 10:
		
		calib_data['mass'] = calib_list[n][0]
  		calib_data['thetax'] = calib_list[n][1]
  		calib_data['thetaz'] = calib_list[n][2]
		calib_data['torque'] = calib_list[n][3]
  		expected_f = GetForces(calib_data['mass'], calib_data['thetax'], calib_data['thetaz'])

		calib_data['fx'] = expected_f[0]
		calib_data['fy'] = expected_f[1]
		calib_data['fz'] = expected_f[2]

		calib_data['frame_width'] = frame_width
		calib_data['frame_height'] = frame_height

		fname = calib_dir+"calib_{:.2f}_{:.2f}_{:.2f}_{:.2f}.json".format(calib_data['mass'], calib_data['thetax'], calib_data['thetaz'], calib_data['torque'])		

		with open(fname, 'w') as fp:
			json.dump(calib_data, fp)

 		sample_count = 0
		n+=1
		if n == len(calib_list): break

		calib_data['MoveData'] = []
		raw_input("working on config: (mass:{:.4f}, thetax:{:.4f}, thetaz:{:.4f}, torque:{:.4f}) - Press any key to collect data".format(calib_list[n][0], calib_list[n][1], calib_list[n][2], calib_list[n][3]))

"""

