#!/usr/bin/python
## ccs, cec24@phy.duke.edu
## push cubes onto wires
import os
import sys
import math
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
## number of cm between wires...
## set this at your own peril...
wireMinDist = 4E-2



if( len(sys.argv) < 2):
    print "# threads centroids onto wires"
    print "# run with: <path_to_centroids_fit.dat> "
    sys.exit(-1)

inpath = sys.argv[1]
infile = os.path.join(inpath, centFile)

print "# binsizeX: " + str(binSizeX) + " binsizeZ: " + str(binSizeZ)

class CubeOid:
    """ a cuboid, has a centroid and an index """
    def __init__(self, index, cx, cy, cz, th):
        self.index = index
        self.cx = cx
        self.cy = cy
        self.cz = cz
        self.th = th
    def write(self):
        retstring = str(self.index) + " " + str(self.cx) + " " + str(self.cy) + " " + str(self.cz) + " " + str(self.th)
        return retstring

        
def dist(this, other):
    dx = math.fabs(this.cx - other.cx)
    dz = math.fabs(this.cz - other.cz)
    return math.sqrt(dx*dx + dz*dz)
    

centList = []

## now we want to bin the centroids by x,z coordinate
xzBinDict = defaultdict(list)


#maxCount = 5
count = 0

centFile = open(infile, 'r')
for line in centFile:
    temp = line.split(" ")
    #print(temp)
    tempCent = CubeOid(int(temp[0]), float(temp[2]), float(temp[3]), float(temp[4]), float(temp[5].rstrip('\n')))
    centList.append(tempCent)
    count = count + 1
    #if(count > maxCount):
    #    break

#print centList    

## bin cubeoids spatially, could really do this while loading them...
for cent in centList:
    binX = round(cent.cx / binSizeX)
    binZ = round(cent.cz / binSizeZ)
#    print "# xbin: " + str(binX) + " zbin: " + str(binZ)
    xzBinDict[(binX, binZ)].append(cent)

## humm not sure this is worked quite right
wireCount = 0
bucketList = []
for k,v in enumerate(xzBinDict):
    print k, v, len(xzBinDict[v])
    # grab the centroids from this bucket
    tempList = xzBinDict[v]
    for cent1 in tempList:
        wireTemp = []
        wireTemp.append(cent1)
        tempList.remove(cent1)
        for cent2 in tempList:
            if(cent1 != cent2):
                distance = dist(cent1, cent2)
                if(distance < wireMinDist):
                    #print '\t', cent1.index, cent2.index, distance
                    wireTemp.append(cent2)
                    tempList.remove(cent2)
                    
        bucketList.append(wireTemp)
        wireCount = wireCount + 1

## this is pretty printing for user
wireCount = 0
for wire in bucketList:
    print "wire: ", wireCount
    for cent in wire:
        print '\t', cent.index, cent.cx, cent.cy, cent.cz, cent.th
    wireCount = wireCount + 1

## lets output to file
outpath = os.path.join(inpath, "wires_list.dat")
outfile = open(outpath, 'w')

outfile.write("# wire_number, cluster_index, cx, cy, cz, theta\n")

wireCount = 0
for wire in bucketList:
    for cent in wire:
        outfile.write(str(wireCount)+ " " + cent.write() + "\n")
    wireCount = wireCount + 1

    

                    
