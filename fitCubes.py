#!/usr/bin/python
## ccs, cec24@phy.duke.edu
## fit cubes run this to actually do the dirty on a set of cluster files
## either uses icp or fit_planes

## clearly these paths are ALL fucked up if you're on windows...
import os
import sys
import subprocess
inpath=""
outpath=""
runBin=""
#runBinICP="./build/src/icpcubes"
#runBinFitPlanes="./build/src/fit_planes"
runBinICP= os.path.join(".", "build" , "src", "icpcubes")
runBinFitPlanes = os.path.join(".", "build", "src", "fit_planes")

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
    print "# using icp " + runBin
else:
    runBin = runBinFitPlanes
    print "# using fitplanes " + runBin

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

nanNormalsList = []
guessNormalsList = []

## return values for when things fuck up
nanNormalsRet = 5
guessNormalsRet = 6

for fileName in cubeFiles:
    runArgs= []
    runArgs.append(runBin)
    runArgs.append(fileName)
    runArgs.append(outpath)
    runArgs.append(indexList[index])
    index = index + 1
    print runArgs

    try: 
        subprocess.check_call(runArgs)
    except subprocess.CalledProcessError, e:
        print "called process error",  e.returncode
        if(e.returncode == nanNormalsRet): # this is a nan normal
            print "# nan normals"
            nanNormalsList.append(runArgs)
        elif(e.returncode == guessNormalsRet): # this is a guessed normal
            print "# guess normals"
            guessNormalsList.append(runArgs)
        else: 
            print "# unknown retcode: ", e.returncode
            sys.exit(-1)
    except:
        print "unhandled error caught"
        sys.exit(-1)
        
    if(index > 5):
        break


## now print out the bad and guess info to log files
nanFile = open("./fitcubes-nanlist.txt", 'w')
for line in nanNormalsList:
    print line
    nanFile.write(line[3] +  " " + line[1] + "\n")
nanFile.close()

guessFile = open("./fitcubes-guesslist.txt", 'w')
for line in guessNormalsList:
    guessFile.write(line[3] +  " " + line[1] + "\n")
guessFile.close()



