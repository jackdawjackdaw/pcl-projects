#include "stdio.h"

#include "constants.h"

#include <iostream>
#include <fstream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

// for safe path stuff
#include <boost/filesystem.hpp>

/**
 * ccs, cec24@phy.duke.edu
 * 11.02.2012
 * 'we happy few'
 * 
 * attempt to use point-cloud-registration to 
 * register a generic cube onto the particular partial
 * one we're looking at.
 * 
 * this is likely to be quite a lot slower than using
 * the normals and edges but it may be more robust.
 *
 */

pcl::PointCloud<pcl::PointXYZ>::Ptr genTestCube(int npointsShortSide, int npointsLongSide);

int main (int argc, char** argv){
	
	if(argc < 3) {
		std::cerr << "# computes transform to rotate a generic cube to a clustered cube " << std::endl;
		std::cerr << "# run with <inputcube..path> <outpath> <id>" << std::endl;
	}
	
	std::string filepath = std::string(argv[1]); 
	std::string outpath = std::string(argv[2]); 
	int id = atoi(argv[3]);
	std::cerr << "# processing: " << filepath << std::endl;
	std::cerr << "# outpath: " << outpath << std::endl;
	std::cerr << "# id: " << id << std::endl;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readBinfileCCS(filepath);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cubeCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile(filepath, *cloud);

	std::cerr << "# read width: " << cloud->width << std::endl;
	std::cerr << "# read height: " << cloud->height << std::endl;

	std::cerr << "# generating cubeCloud" << std::endl;
	// low res
	cubeCloud = genTestCube(9,27); // gen our comparison cube
	// higher res
	//cubeCloud = genTestCube(18,54); // gen our comparison cube
	
	std::cerr << "# starting icp" << std::endl;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cubeCloud);
  icp.setInputTarget(cloud);
	icp.setTransformationEpsilon(1e-6);
	icp.setMaximumIterations(128);
	icp.setRANSACOutlierRejectionThreshold(0.01);
	//icp.setMaxCorrespondenceDistance(0.01);

  icp.align(*Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
	std::stringstream ss;
	
	ss << "fitted_cube_" << id << ".pcd";
	boost::filesystem::path outPath(outpath); // append the result string to the outpath correctly
	outPath /= ss.str();
	std::cerr << "# filename: " << outPath << std::endl;
	//writeBinfileCCS(Final, outpath);
	pcl::io::savePCDFileASCII (outPath.native(), *Final);

	std::ofstream file;
	ss.clear();
	ss.str("");
	ss << "transformation_" << id << ".txt";
	outPath.clear();
	outPath /= outpath;
	outPath /= ss.str();
	std::cerr << "# filename: " << outPath << std::endl;
	file.open(outPath.c_str());
	file << icp.getFinalTransformation() << std::endl;
	file.close();
	
	return EXIT_SUCCESS;
}


pcl


/**
 * generate a cuboid centered at the origin
 * 
 * the x,z planes have npointsShort per side
 * the y,z and y,z planes have npointsLong per side
 * the side lengths are set at cubeShortSide and cubeLongSide
 * repsectively
 * 
 * by eye there are ~ 18 scan pts on a short side,
 * so we should have 54 on a long side
 */

pcl::PointCloud<pcl::PointXYZ>::Ptr genTestCube(int npointsShortSide, int npointsLongSide){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cubeCloud (new pcl::PointCloud<pcl::PointXYZ>);


	int npointsBottomFace = npointsShortSide* npointsShortSide;
	int npointsSideFace = npointsShortSide* npointsLongSide;
	int npointsTotal = 2*npointsBottomFace + 4 *npointsSideFace;
	
	float dxShortSide = cubeShortSide / (float)npointsShortSide;
	float dxLongSide =  cubeLongSide /  (float)npointsLongSide;
	
	std::cerr << "# dxShortSide: " << dxShortSide << std::endl;
	std::cerr << "# dxLongSide: " << dxLongSide << std::endl;
	std::cerr << "# npointsTotal: " << npointsTotal << std::endl;
	
	cubeCloud->width = npointsTotal;
	cubeCloud->height = 1;
	cubeCloud->points.resize(npointsTotal); // allocate space for all the points we need
	
	// make the top and bottom cap faces
	// these go at y = +- cubeLongSide /2 
	// from x,z = -ls/2, -ls/2  to x,z = ls/2, ls/2
	int counter = 0;
	float xval, yval, zval;
	float xOffset, yOffset, zOffset;
	
	// top face
	yval = cubeLongSide / 2;
	xOffset = - cubeShortSide / 2;
	zOffset = - cubeShortSide / 2;
	for(int i = 0; i < npointsShortSide; i++){
		for(int j = 0; j < npointsShortSide; j++){
			cubeCloud->points[counter].x = i*dxShortSide +  xOffset;
			cubeCloud->points[counter].y = yval;
			cubeCloud->points[counter].z = j*dxShortSide + zOffset;
			counter++;
		}
	}

	// bottom face
	yval = -cubeLongSide / 2;
	xOffset = - cubeShortSide / 2;
	zOffset = - cubeShortSide / 2;
	for(int i = 0; i < npointsShortSide; i++){
		for(int j = 0; j < npointsShortSide; j++){
			cubeCloud->points[counter].x = i*dxShortSide + xOffset;
			cubeCloud->points[counter].y = yval;
			cubeCloud->points[counter].z = j*dxShortSide + zOffset;
			counter++;
		}
	}


	// make each side plane
	// 1) z= -cubeShortSide/2, x: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2
	// 2) z= cubeShortSide/2, x: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2
	// 3) x = -cubeShortSide/2, z: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2
	// 4) x = -cubeShortSide/2, z: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2


	zval = -cubeShortSide / 2;
	xOffset = - cubeShortSide / 2;
	yOffset = - cubeLongSide / 2;
	for(int i = 0; i < npointsShortSide; i++){
		for(int j = 0; j < npointsLongSide; j++){
			cubeCloud->points[counter].x = i*dxShortSide + xOffset;
			cubeCloud->points[counter].y = j*dxLongSide + yOffset;
			cubeCloud->points[counter].z = zval;
			counter++;
		}
	}

	zval = cubeShortSide / 2;
	xOffset = - cubeShortSide / 2;
	yOffset = - cubeLongSide / 2;
	for(int i = 0; i < npointsShortSide; i++){
		for(int j = 0; j < npointsLongSide; j++){
			cubeCloud->points[counter].x = i*dxShortSide + xOffset;
			cubeCloud->points[counter].y = j*dxLongSide +  yOffset;
			cubeCloud->points[counter].z = zval;
			counter++;
		}
	}

	xval = -cubeShortSide / 2;
	zOffset = - cubeShortSide / 2;
	yOffset = - cubeLongSide / 2;
	for(int i = 0; i < npointsShortSide; i++){
		for(int j = 0; j < npointsLongSide; j++){
			cubeCloud->points[counter].x = xval;
			cubeCloud->points[counter].y = j*dxLongSide + yOffset;
			cubeCloud->points[counter].z = i*dxShortSide + zOffset;
			counter++;
		}
	}

	xval = cubeShortSide / 2;
	zOffset = - cubeShortSide / 2;
	yOffset = - cubeLongSide / 2;
	for(int i = 0; i < npointsShortSide; i++){
		for(int j = 0; j < npointsLongSide; j++){
			cubeCloud->points[counter].x = xval;
			cubeCloud->points[counter].y = j*dxLongSide + yOffset;
			cubeCloud->points[counter].z = i*dxShortSide + zOffset;
			counter++;
		}
	}
	

	return cubeCloud;
}
