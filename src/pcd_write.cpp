#include "stdio.h"
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// for safe path stuff
#include <boost/filesystem.hpp>

/**
 * reads dumped file of xyz in from file-path
 * these are the world space coordinates computed 
 * by the camera / projector pair
 * 
 * outputs 4 xyz bin and pcd filesx
 */

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

	int width, height;
	int nPoints;
	float *pointsX, *pointsY, *pointsZ;
	float px, py, pz;

	if(argc < 2){
		std::cerr << "# args: infile.dat, outpath" << std::endl;
		return EXIT_FAILURE;
	}

	std::string outpathInput = argv[2];


	fprintf(stderr, "# reading data from file: %s\n", argv[1]);

	FILE *fptr = fopen(argv[1], "r");
	if(fptr == NULL){
		fprintf(stderr, "# bad input file: %s\n", argv[1]);
		return EXIT_FAILURE;
	}
	
	fread(&nPoints, sizeof(int), 1, fptr);
	fread(&height, sizeof(int), 1, fptr);
	fread(&width, sizeof(int), 1, fptr);

	fprintf(stderr, "# npoints %d\n", nPoints );  	
	fprintf(stderr, "# width: %d height: %d\n", width, height );  
		
	pointsX = (float*)malloc(sizeof(float)*nPoints);
	pointsY = (float*)malloc(sizeof(float)*nPoints);
	pointsZ = (float*)malloc(sizeof(float)*nPoints);
	
	for(int i = 0; i < nPoints; i++){
		fread(&px, sizeof(float), 1, fptr);
		fread(&py, sizeof(float), 1, fptr);
		fread(&pz, sizeof(float), 1, fptr);
		pointsX[i] = px;
		pointsY[i] = py;
		pointsZ[i] = pz;
	}
	
	fclose(fptr);

	int nclouds = 4;
	int offset = 0;
	int cloudSize = nPoints/nclouds;
	std::stringstream ss;

	for(int index = 0; index < nclouds; ++index){
		// Fill in the cloud data
		// we know this from the pfm file
		offset = index*cloudSize;
		cloud.width    = cloudSize;
		cloud.height   = 1;
		cloud.is_dense = false;
		cloud.points.resize (cloud.width * cloud.height);
		for (size_t i = 0; i < cloud.points.size (); ++i)
			{
				cloud.points[i].x = pointsX[i+offset];
				cloud.points[i].y = pointsY[i+offset];
				cloud.points[i].z = pointsZ[i+offset];
			}


		ss << "cloud_cam_" << index << ".pcd";
		boost::filesystem::path outPath(outpathInput); // append the result string to the outpath correctly
		outPath /= ss.str();
		pcl::io::savePCDFileASCII (outPath.native(), cloud);

		std::cerr << "Saved " << cloud.points.size () << " data points to: " << outPath.native() << std::endl;
		ss.clear(); // reset the stringstream
		ss.str("");
		
	}

	// and write out the whole thing too
	cloudSize = nPoints;
	cloud.width    = cloudSize;
	cloud.height   = 1;
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);
	for (size_t i = 0; i < cloud.points.size (); ++i)
		{
			cloud.points[i].x = pointsX[i];
			cloud.points[i].y = pointsY[i];
			cloud.points[i].z = pointsZ[i];
		}

	ss << "cloud_full.pcd";
	boost::filesystem::path outPath(outpathInput); // append the result string to the outpath correctly
	outPath /= ss.str();
	#ifdef PCDWRITEBINFILE
	pcl::io::savePCDFileBinary (outPath.native(), cloud);
	#else 
	pcl::io::savePCDFileASCII (outPath.native(), cloud);
	#endif

	std::cerr << "Saved " << cloud.points.size () << " data points to: " << outPath.native() << std::endl;
	ss.clear(); // reset the stringstream
	ss.str("");

	free(pointsX);
	free(pointsY);
	free(pointsZ);

  return (0);
}
