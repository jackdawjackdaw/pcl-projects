
#include "stdio.h"
#include "constants.h"

#include <iostream>
#include <locale>
#include <string>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
// need these two to do the rotations 
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <boost/filesystem.hpp>


void rotateAndOutputCube(std::string outpath, std::string basename, 
												 pcl::PointCloud<pcl::PointXYZ>::Ptr cubeCloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr genTestCube(int npointsShortSide, int npointsLongSide, int sideArray[6]);


/**
 * ccs, 
 * 
 * make a test cube, and apply a series of 6 rotations to it, saving
 * each cube into the folder outpath. We only need to rotate up to pi/2 because
 * our cloud is in S4 / SO(2) right?
 *
 * this is useful for testing the normal and rotation extraction methods
 *
 * the cubes are translated to (1.0,0,0)
 *
 * 'you're not a good shot but i'm worse'
 */

int main (int argc, char** argv){
	
	if(argc < 2) {
		std::cerr << "# makes some test cubes for fit_planes.cpp" << std::endl;
		std::cerr << "# run with <outpath>" << std::endl;
		return EXIT_FAILURE;
	}

	std::string outpath = std::string(argv[1]); 

	pcl::PointCloud<pcl::PointXYZ>::Ptr cubeCloud;
	std::cerr << "# outputto: " << outpath << std::endl;

	std::cerr << "# generating cubeCloud" << std::endl;

	
	// apply a series of rotations and output the cube
	// then create a cube with only sides no tops
	// then only tops no sides
	// then random combinations
	// etc..
	int sideArray[6] = {1,1,1,1,1,1};
	std::cerr << "# sideArray: ";
	for(int i = 0; i < 6; i ++)
		std::cerr << sideArray[i] << " ";
	std::cerr << std::endl;
	
	cubeCloud = genTestCube( nShortSideLowRes, nLongSideLowRes, sideArray);

	rotateAndOutputCube(outpath, "full-cube", cubeCloud);

	// now we'll repeat without the horizontal endcaps
	sideArray[0] = 0;
	sideArray[1] = 0;
	cubeCloud = genTestCube( nShortSideLowRes, nLongSideLowRes, sideArray);
	rotateAndOutputCube(outpath, "no-horizontals", cubeCloud);

	// reset the cubeCloud?
	cubeCloud->clear();

	// now repeat with only horizontals
	sideArray[0] = 1;
	sideArray[1] = 1;
	sideArray[2] = 0;
	sideArray[3] = 0;
	sideArray[4] = 0;
	sideArray[5] = 0;
	cubeCloud = genTestCube(nShortSideLowRes, nLongSideLowRes, sideArray);
	rotateAndOutputCube(outpath, "no-verticals", cubeCloud);

	cubeCloud->clear();


	// now repeat with a mix
	sideArray[0] = 1;
	sideArray[1] = 0;
	sideArray[2] = 0;
	sideArray[3] = 1;
	sideArray[4] = 0;
	sideArray[5] = 1;
	cubeCloud = genTestCube(nShortSideLowRes, nLongSideLowRes, sideArray);
	rotateAndOutputCube(outpath, "mixed-cube", cubeCloud);

	

	return EXIT_SUCCESS;

}


void rotateAndOutputCube(std::string outpath, std::string basename, 
												 pcl::PointCloud<pcl::PointXYZ>::Ptr cubeCloud){

	// filesystem safe paths
	boost::filesystem::path outPathFull(outpath); // append the result string to the outpath correctly

	int nrot = 6;
	float dtheta = (M_PI / (2*(float)nrot)); // angle increments to rotate by
	std::stringstream ss;
	float theta = 0.0;
	Eigen::Affine3f transMat;

	float translation = 1.0;


	pcl::PointCloud<pcl::PointXYZ>::Ptr cubeRotated  (new pcl::PointCloud<pcl::PointXYZ>);
	
	for(int i = 0; i < nrot; i++){
		theta = (float)i * dtheta;

		std::cerr << "# theta: " << theta << std::endl;
			
		ss << basename << "-" << i << "-" << theta << ".pcd";
		outPathFull /= ss.str();

		std::cerr << "# writing: " << outPathFull.native() << std::endl;
		
		// generate the rotation, 
		transMat = pcl::getTransformation(translation,0.0,0.0, 0.0, theta, 0.0);
		
		// apply the rotation
		pcl::transformPointCloud(*cubeCloud, *cubeRotated, transMat);
		
		pcl::io::savePCDFileASCII(outPathFull.native(), *cubeRotated);

		// reset the strings
		outPathFull.clear();
		ss.clear();
		ss.str("");
		outPathFull /= outpath;
	}
	
}

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
 *
 * sideArray turns on the various sides {1,1,1,1,1,1} means we have all of them
 * numbered as in the fn below (top, bottom, 1, 2, 3, 4)
 * and so on
 */

pcl::PointCloud<pcl::PointXYZ>::Ptr genTestCube(int npointsShortSide, int npointsLongSide, int sideArray[6]){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cubeCloud (new pcl::PointCloud<pcl::PointXYZ>);



	int npointsBottomFace = npointsShortSide* npointsShortSide;
	int npointsSideFace = npointsShortSide* npointsLongSide;
	int npointsTotal = 0;
	
	int nface = 0;

	for(int i = 0; i < 6;i++){
		if(i < 2){
			nface = npointsBottomFace;
		} else {
			nface = npointsSideFace;
		}
		npointsTotal += sideArray[i] * nface;
	}


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

	if(sideArray[0]){
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
	}

	if(sideArray[1]){
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
	}


	// make each side plane
	// 1) z= -cubeShortSide/2, x: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2
	// 2) z= cubeShortSide/2, x: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2
	// 3) x = -cubeShortSide/2, z: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2
	// 4) x = -cubeShortSide/2, z: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2

	if(sideArray[2]){
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
	}

	if(sideArray[3]){
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
	}

	if(sideArray[4]){
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
	}

	if(sideArray[5]){
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
	}
	

	return cubeCloud;
}
