#!/usr/bin/python
## ccs, cec24@phy.duke.edu
## push cubes onto wires
import os
import sys
from collections import defaultdict

centFile = "centroids_fit.dat"

## the min /  max ranges for the centroid cloud
xminCloud = -3
xmaxCloud = 3
zminCloud = -3
zmaxCloud = 3 
## how many spatial bins? 32 say
nbinsX = 32
nbinsZ = 32
## binsize
binSizeX = float(xmaxCloud - xminCloud) / nbinsX
binSizeZ = float(zmaxCloud - zminCloud) / nbinsZ



if( len(sys.argv) < 2):
    print "# threads centroids onto wires"
    print "# run with: <path_to_centroids_fit.dat>"
    sys.exit(-1)

inpath = sys.argv[1]
infile = os.path.join(inpath, centFile)

print "# binsizeX: " + str(binSizeX) + " binsizeZ: " + str(binSizeZ)

class CubeOid:
    """ a cuboid, has a centroid and an index """
    def __init__(self, index, cx, cy, cz):
        self.index = index
        self.cx = cx
        self.cy = cy
        self.cz = cz
    

centList = []
wireList = []

## now we want to bin the centroids by x,z coordinate
xzBinDict = defaultdict(list)


#maxCount = 5
count = 0

centFile = open(infile, 'r')
for line in centFile:
    temp = line.split(" ")
    print(temp)
    tempCent = CubeOid(int(temp[0]), float(temp[2]), float(temp[3]), float(temp[4]))
    centList.append(tempCent)
    count = count + 1
    #if(count > maxCount):
    #    break

print centList    

## bin cubeoids spatially, could really do this while loading them...
for cent in centList:
    binX = round(cent.cx / binSizeX)
    binZ = round(cent.cz / binSizeZ)
    print "# xbin: " + str(binX) + " zbin: " + str(binZ)
    xzBinDict[(binX, binZ)].append(cent)

## humm not sure this is worked quite right
for k,v in enumerate(xzBinDict):
    print k, v, len(xzBinDict[v])

    
