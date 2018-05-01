#!/usr/bin/python
#\file    optimize.py
#\date    Apr.4, 2018
#
#coordinates:
#
#                      +z ^ (finger tip)
#                         |
#+y(down into finger tip) o--> +x (finger right side)
import matplotlib
matplotlib.use('Agg')

import math
import numpy as np
import numpy.linalg as la
import json
import glob
import matplotlib.pyplot as plt
import argparse
import sys

from sklearn.decomposition import PCA
from math import cos, sin
from operator import sub

calib_dir = '../calib/'

frame_height = 0
frame_width = 0
#frame data: Pox, Poy, So, Dpx, Dpy, DS

dx = []
dy = [] # sqrt(dx^2 + dz^2)
dz = []
fx = [] # expected values
fy = []
fz = []

def blob_to_data_norm(bms):
    global dx, dy, dz
    
    dy += [0]
    dx += [0]
    dz += [0]

    for mv in bms:
        dy[-1] += la.norm((mv[3], mv[4]))
        dx[-1] += mv[3]
        dz[-1] += mv[4]

    dy[-1] = dy[-1]/len(bms)
    dx[-1] = dx[-1]/len(bms)
    dz[-1] = dz[-1]/len(bms)

    return
 
def blob_to_data_dist(bms):
    global dx, dy, dz
    
    meanx = 0
    meanz = 0
    meany = 0

    for mv in bms:
        meanx += mv[3]
        meanz += mv[4]

    meanx/=len(bms)
    meanz/=len(bms)
     
    for mv in bms:
        meany += ((meanx-mv[3])**2 + (meanz-mv[4])**2)#la.norm(((dx[-1]-mv[3]),(dz[-1]-mv[4])))

    meany = np.sqrt(meany/len(bms))#dy[-1]/len(bms)

    dy += [meany]
    dx += [meanx]
    dz += [meanz]
    
    return
 
def blob_to_data_pca(bms):
    global dx, dy, dz
    
    dx += [0]
    dz += [0]
    dx_local = []
    dz_local = []

    for mv in bms:
        dx[-1] += mv[3]
        dz[-1] += mv[4]
        dx_local += [mv[3]]
        dz_local += [mv[4]]

    dx[-1] = dx[-1]/len(bms)
    dz[-1] = dz[-1]/len(bms)
    
    X = np.column_stack((np.array(dx_local), np.array(dz_local)))
    
    pca = PCA(n_components=2)
    pca.fit(X)
    
   # mean = pca.mean_ / la.norm(pca.mean_)
    
   # comp1 = pca.components_[0] / la.norm(pca.components_[0])
   # comp2 = pca.components_[1] / la.norm(pca.components_[1])
    
   # if np.dot(mean, comp1) < np.dot(mean, comp2):
   #     dy += [np.sqrt(pca.explained_variance_[0])]
   # else:
   #     dy += [np.sqrt(pca.explained_variance_[1])]
    
    dy += [np.sqrt(pca.explained_variance_[0])]

    return

if __name__=='__main__':
    
    convert_f = {"norm":blob_to_data_norm, "dist":blob_to_data_dist, "pca":blob_to_data_pca}

    parser = argparse.ArgumentParser()
    parser.add_argument("--fy_mode", '-y', help="Modes for calculating fy:\n\"norm\" for norm mode\n\"dist\" for dist from mean mode\n\"pca\" for pca mode", action="store", dest="fymode", nargs=1, type=str, required=True)
    args = parser.parse_args()

    fymode = args.fymode[0]
    
    for file in glob.glob(calib_dir+'*.json'):
        calib_data = json.load(open(file))

        expected = [calib_data["fx"], calib_data["fz"], calib_data["fy"]]


        for blob_move in calib_data["MoveData"]:
            fy += [expected[2]]
            fx += [expected[0]]
            fz += [expected[1]]
            
            convert_f[fymode](blob_move["BlobMoves"])

    dx = np.asarray(dx)
    dy = np.asarray(dy)
    dz = np.asarray(dz)
    fx = np.asarray(fx)
    fy = np.asarray(fy)
    fz = np.asarray(fz)

    Ax = np.vstack([dx, np.ones(len(dx))]).T
    mx, cx = np.linalg.lstsq(Ax, fx)[0]
    print ("m, c values for x direction: ["+str(mx)+str(cx)+"]")

    Ay = np.vstack([dy, np.ones(len(dy))]).T
    (my, cy) = np.linalg.lstsq(Ay, fy)[0]
    print ("m, c values for y direction: ["+str(my)+str(cy)+"]")
    
    Az = np.vstack([dz, np.ones(len(dz))]).T
    (mz, cz) = np.linalg.lstsq(Az, fz)[0]
    print("m, c values for z direction: ["+str(mz)+","+str(cz)+"]")
    
    plt.figure(1)
    plt.suptitle("Fitting with method: "+fymode)
    
    plt.subplot(221)
    plt.title("fx fitting")
    plt.xlabel("dx")
    plt.ylabel("fx (N)")
    plt.plot(dx, fx, 'o', markersize=2)
    plt.plot(dx, mx*dx + cx, 'r')
    
    plt.subplot(222)
    plt.title("fz fitting")
    plt.xlabel("dz")
    plt.ylabel("fz (N)")
    plt.plot(dz, fz, 'o', markersize=2)
    plt.plot(dz, mz*dz + cz, 'g')
    
    plt.subplot(223)
    plt.title("fy fitting")
    if fymode == "norm":
        plt.xlabel("dy")
    else:
        plt.xlabel("stdev of distance from shear center")
    plt.ylabel("fy (N)")
    #plt.text(0, -5, "(m,c): ({:.5f},{:.5f})".format(my, cy))
    plt.plot(dy, fy, 'o', markersize=2)
    plt.plot(dy, my*dy + cy, 'b', label='Fitted line')
    
#    plt.subplot(224)
#    plt.title("dx vs dz")
#    plt.xlabel("dx")
#    plt.ylabel("dz")
#    plt.plot(dx, dz, 'o', markersize=2)
    plt.savefig("..\scatter_plots\optimize_"+fymode+".png")
    plt.show()