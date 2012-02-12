#!/usr/bin/python
## fit cubes
import os
inpath="./test-data/clusters/"
outpath="./test-data/fitted-cubes/"
runBin="./build/src/icpcubes"


cubeFiles = []
indexList = []

for dirname, dirnames, filenames in os.walk(inpath):
    for filename in filenames:
        #print os.path.join(dirname, filename)
        # ugly partitioning
        prefix=filename.rpartition("_")
        f=prefix[2].partition(".")
        index=f[0]
        print(index)
        indexList.append(index)
        cubeFiles.append(os.path.join(dirname, filename))

index = 0
for fileName in cubeFiles:
    runstring = runBin + " " + fileName + " " + outpath + " " + indexList[index]
    index = index + 1
    print runstring
    #os.system(runstring) 



