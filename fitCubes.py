#!/usr/bin/python
## fit cubes
## either uses icp or fit_planes

## clearly these paths are ALL fucked up if you're on windows...
import os
import sys
inpath=""
outpath=""
runBin=""
runBinICP="./build/src/icpcubes"
runBinFitPlanes="./build/src/fit_planes"

if (len(sys.argv) < 4):
    print "# attempts to extract centroids and angles from all files in <inpath>"
    print "# output is written to <outpath>"
    print "# binflag: 1 -> icp"
    print "#        : else -> fitplanes"
    print "# run with: <inpath> <outpath> <binFlag>"
    sys.exit(-1)


inpath = sys.argv[1]
outpath = sys.argv[2]

if(sys.argv[3] == "1"):
    runBin = runBinICP
    print "# using icp"
else:
    runBin = runBinFitPlanes
    print "# using fitplanes"

## check that the bin we want to use exists (durp)
if (os.path.isfile(runBin) == False):
    print "# cannot find binfile: " + runBin
    sys.exit(-1)
    

    

print "# reading input from: " + inpath
print "# writing output to: " + outpath

cubeFiles = []
indexList = []

for dirname, dirnames, filenames in os.walk(inpath):
    for filename in filenames:
        #print os.path.join(dirname, filename)
        # ugly partitioning
        prefix=filename.rpartition("_")
        f=prefix[2].partition(".")
        index=f[0]
        #print(index)
        indexList.append(index)
        cubeFiles.append(os.path.join(dirname, filename))

index = 0
for fileName in cubeFiles:
    runstring = runBin + " " + fileName + " " + outpath + " " + indexList[index]
    index = index + 1
    print runstring
    os.system(runstring) 



