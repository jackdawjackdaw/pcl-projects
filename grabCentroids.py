#!/usr/bin/python
# grab the fourth col of each file and 
# just print the last bit
import os

inpath="./test-data/fitted-cubes/"

for dirname, dirnames, filenames in os.walk(inpath):
    #print(dirname)
    for filename in filenames:
        name,ext = os .path.splitext(filename)
        if(ext == ".txt"):
            #print(filename)
            f = open(os.path.join(dirname, filename), 'r')
            coords = []
            for i in [1,2,3]:
                z = f.readline().split() # read first line
                coords.append(z[-1])
            print str(coords[0]) + " " + str(coords[1]) + " " + str(coords[2])
        
