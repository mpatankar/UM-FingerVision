# -*- coding: utf-8 -*-
"""
Created on Sun Apr 22 23:57:33 2018

@author: Justin
"""
import matplotlib
matplotlib.use('Agg')

import math
import numpy as np
import numpy.linalg as la
import json
import glob
import matplotlib.pyplot as plt

from math import cos, sin
from operator import sub
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler

calib_dir = '../calib/'

frame_height = 0
frame_width = 0
#frame data: Pox, Poy, So, Dpx, Dpy, DS

dx = []
dy = [] # sqrt(dx^2 + dz^2)
dz = []
fx = []
fy = []
fz = []

variance_thres = 0

def collect_shear(mv):
    global dx, dz

    dx += [mv[3]]
    dz += [mv[4]]

    return

def calc_force(blob_moves):
    for move in blob_moves:
        collect_shear(move)
    
def draw_vector(v0, v1, ax=None):
    ax = ax or plt.gca()
    arrowprops=dict(arrowstyle='->',
                    linewidth=2,
                    shrinkA=0, shrinkB=0)
    ax.annotate('', v1, v0, arrowprops=arrowprops)
        
if __name__=='__main__':
    
    n=1
    for file in glob.glob(calib_dir+'*.json'): # [calib_dir+"calib_135.50_0.00_15.00_0.00.json"]:
        calib_data = json.load(open(file))
        
     
        for blob_move in calib_data["MoveData"]:
            calc_force(blob_move["BlobMoves"])
            
            if n % 10 == 0:
               
                X = np.column_stack((np.array(dx), np.array(dz)))
                #X_std = StandardScaler().fit_transform(X)
                pca = PCA(n_components=2)
                pca_result = pca.fit(X)
                
                plt.figure(n)
                plt.title(str(file)+" sample (PCA) "+str(n)+"\nExpected (fx, fy, fz): ({:.4f},{:.4f},{:.4f})".format(calib_data["fx"],calib_data["fy"],calib_data["fz"]))
                plt.xlabel("dx")
                plt.ylabel("dz")
                plt.plot(X[:,0], X[:,1], 'o', markersize=2)
                for length, vector in zip(pca.explained_variance_, pca.components_):
                    v = vector * np.sqrt(length)
                    draw_vector(pca.mean_, pca.mean_ + v)
                plt.axis('equal')
                plt.text(0,0,"mean: [{:.4f},{:.4f}]\nstdev: [{:.4f},{:.4f}]".format(pca.mean_[0],pca.mean_[1],pca.explained_variance_[0], pca.explained_variance_[1]))
                plt.savefig("..\scatter_plots\pca"+file[14:len(file)-5]+".png")
                
            n+=1
            dx = []
            dy = []
            dz = []
            fx = []
            fy = []
            fz = []

        dx = np.asarray(dx)
        dy = np.asarray(dy)
        dz = np.asarray(dz)
        fx = np.asarray(fx)
        fy = np.asarray(fy)
        fz = np.asarray(fz)

#        plt.savefig("..\scatter_plots\scatter"+file[14:len(file)-5]+".png")