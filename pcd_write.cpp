#include "stdio.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/**
 *  reads dumped file of xyz in from file-path
 * these are the world space coordinates computed 
 * by the camera / projector pair
 */

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

	int width, height;
	int nPoints;
	float *pointsX, *pointsY, *pointsZ;
	float px, py, pz;

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

  // Fill in the cloud data
	// we know this from the pfm file
  cloud.width    = nPoints/4;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = pointsX[i];
    cloud.points[i].y = pointsY[i];
    cloud.points[i].z = pointsZ[i];
  }

  pcl::io::savePCDFileASCII ("test_pcd_1.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  // for (size_t i = 0; i < cloud.points.size (); ++i)
  //   std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

	free(pointsX);
	free(pointsY);
	free(pointsZ);

  return (0);
}
