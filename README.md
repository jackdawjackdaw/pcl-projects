MEADRE
=======

working with openPCL for point cloud segmentation

depends
-------
* openPCL
* boost (for file-system libs)

setup
-------
* pull/clone
* mkdir ./build; cd ./build
* cmake ..
* make
* there's currently no install script, bins will be in ./build/src/


work flow:
------

1) obtain point clouds from camera / projector system input file. 

2) inspect constants.h, the values cubeLongSide and cubeShortSide are used throughout the analysis, they should be roughly correct or fit_planes will struggle. 
	 
3) using cluster (Args: <cloud_file.pcd> <outpath>) segment the full point cloud into separate clouds each one representing
	 a cube. Each cloud is output as a pcd file, pcd_viewer is really useful for debugging. Clusters are indexed in the order they're pulled out of the original data set. This index is carried through for the rest of the analysis.

4) run fitCubes.py, Args: <path_to_clusters> <path_to_output>. This will default to using fit_planes to extract the centroids and angles. Several ascii data files are created in the output path: 
- centroids_fit.dat: each line is cluster_index, centroid.x, centroid.y, centroid.z, angle.
	angles are reported mod PI/2 (since we're dealing with cubes)
- fitCubes-guesslist.txt, lists clusters which only had a single face, so the normals had to be guessed, centroid may be ok, but angle is likely to be off. Each line is: cluster_id, path_to_cluster_pcd_file
- fitCubes-nanlist.txt, lists clusters which totally failed, it's worth using pcd-viewer to check these out. Each line is: cluster_id, path_to_cluster_pcd_file
- fitCubes-recon-failed.txt, more failed clusters, these failed because the cluster sizes are too small for planar model fitting, each line: cluster_id, path_to_cluster_pcd_file
- fitCubes-unknown.txt, clusters which failed for who knows why? each line: cluster_id, path_to_cluster_pcd_file, return val

5) run wireFinder, Args <path_to_centroids_fit.dat> <outpath> will output wirelist.txt a file which lists which clusters belong to which wires, each line: wire_id cluster_id

file summary:
------
pcd_write -> converts input binfile containing xyz locations of voxels into a pcd file this forms the		
basis for the rest of the analysis, the pcd file can be set to binary by defining PCDWRITEBINFILE.
input file is expected to be: <int> npoints, <int> height, <int> width, npoints*{float x float y float z}.
							
cluster_extract -> reads a pcdfile and an outpath, pulls out euclidean clusters. Clusters are output to outpath. A text file with centroid guesses computed by: pcl::compute3DCentroid is created.
				
fit_planes -> fits planar models to an extracted cluster, planes are saved to file (for debugging), normals and centroid for the cube or whatever we have of it are extracted. The y rotation mod M_PI is also extracted.
				
pcd2bin -> convert a pcd file to a simple bin file which can be read using the binfile.h routines, useful if you're suffering from boost string problems on os-x 
bin2pcd -> convert a bin file back to a pcd file.

icpcubes -> make a test cube, use icp to register this cube with a supplied cluster file, outputs the 
registered test cube and the transformation matrix (in ascii)
				
make_test_cubes -> creates a set of test (partial) cubes and applies rotations around the y axis, 
useful to test fit_planes and icpcubes
