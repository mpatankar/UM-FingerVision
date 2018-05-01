#!/usr/bin/python
#\file    vs_test_force.py
#\brief   Test code of FingerVision (force estimation from marker (blob) movement)
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Apr.18, 2017
import roslib#; roslib.load_manifest('lfd_vision')
import rospy
import fv_standalone.msg
import math
import numpy as np
import numpy.linalg as la

import argparse

def FListToStr(flist, fmt='%0.2f'):
  return '['+' '.join(map(lambda f:fmt%f, flist))+']'

def blob_to_force_norm(msg):
    cx= msg.width/2
    cy= msg.height/2
    div= float(msg.width+msg.height)/2.0

    fscale = [0.79239372719359247, 7.5093060352789971, 1.1836919827372239]
    posforce_array = []

    for mv in msg.data:
        posforce_array += [[-(mv.Pox-cx)/div, -(mv.Poy-cy)/div,
			-fscale[0]*mv.DPx, 
                      fscale[1]*la.norm((mv.DPx, mv.DPy)), 
                        -fscale[2]*mv.DPy]]

    def convert_wrench(p_f):
        p,f= p_f[:2], p_f[2:]
        tscale= 1.0
        tau= np.cross([p[0],0.0,p[1]],f)*tscale
        return f+tau.tolist()
    
    wrench_array= [convert_wrench(p_f) for p_f in posforce_array]
    wrench_avr= [sum([wrench[d] for wrench in wrench_array])/float(len(wrench_array)) for d in xrange(6)]

    print 'Wrench(average)=', FListToStr(wrench_avr)

    return wrench_avr
 
def blob_to_force_dist(msg):
    fscale = [0.79239372719359247, 2.50915, 1.1836919827372239]
    
    dy = 0
    dx = 0
    dz = 0

    for mv in msg.data:
        dx += mv.DPx
        dz += mv.DPy

    dx = dx/len(msg.data)
    dz = dz/len(msg.data)
    
    for mv in msg.data:
        dy += ((dx-mv.DPx)**2 + (dz-mv.DPy)**2)#la.norm(((dx-mv.DPx),(dz-mv.DPy)))

    dy = np.sqrt(dy/len(msg.data))#dy/len(msg.data)
    force = [fscale[0]*dx, fscale[1]*dy-0.454, fscale[2]*dz, dy]
    print "force(average+stdev_y=", FListToStr(force)
    return force
    
def blob_to_force_pca(msg):
    fscale = [1, 33.358014866569484, 1]
   
    print "pca" 
    return ["pca"]
    
def BlobMoves(msg):
  cx= msg.width/2
  cy= msg.height/2
  div= float(msg.width+msg.height)/2.0
  #Convert marker movement to marker position (p) and 3D force (f)
  def convert_raw(mv):
    fscale= [1.0,1.0,1.0]
    rxy= ((mv.Pox-cx)/div, (mv.Poy-cy)/div)
    fxy= (mv.DPx, mv.DPy)
    fz= la.norm(fxy) # max(0.0,mv.DS)
    #f= [+(fxy[0]*fscale[0]), -(fz*fscale[2]), -(fxy[1]*fscale[1])]
    #p= [+rxy[0], -rxy[1]]
    f= [-(fxy[0]*fscale[0]), +(fz*fscale[2]), -(fxy[1]*fscale[1])]
    p= [-rxy[0], -rxy[1]]
    return p+f
  #Convert position (p) and 3D force (f) to wrench (force + torque)
  def convert_wrench(p_f):
    p,f= p_f[:2], p_f[2:]
    tscale= 1.0
    tau= np.cross([p[0],0.0,p[1]],f)*tscale
    return f+tau.tolist()
  posforce_array= [convert_raw(mv) for mv in msg.data]
  wrench_array= [convert_wrench(p_f) for p_f in posforce_array]
  #Average wrench
  wrench_avr= [sum([wrench[d] for wrench in wrench_array])/float(len(wrench_array)) for d in xrange(6)]
  print 'Wrench(average)=', FListToStr(wrench_avr)

if __name__=='__main__':
    convert_f = {"norm":blob_to_force_norm, "dist":blob_to_force_dist, "pca":blob_to_force_pca}
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--fy_mode", '-y', help="Modes for calculating fy:\n\"norm\" for norm mode\n\"dist\" for dist from mean mode\n\"pca\" for pca mode", action="store", dest="fymode", nargs=1, type=str, required=True)
    args = parser.parse_args()
    
    fymode = args.fymode[0]
    
    rospy.init_node('fv_test_force')
    
    #Subscribing blob movement topic (marker tracking)
    sub_bm= rospy.Subscriber("fv_standalone/blob_moves",
                           fv_standalone.msg.BlobMoves, convert_f[fymode])
    
    rospy.spin()
  
