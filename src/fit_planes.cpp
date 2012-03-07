
#include "stdio.h"
#include <iostream>
#include <locale>
#include <string>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/octree/octree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <pcl/surface/concave_hull.h>

// for portable filepaths
#include <boost/filesystem.hpp>

// get the cube dims and other constants
#include "constants.h"
// defines a plane, used a lot here
#include "planeinfo.h"

//#define DEBUG

/**
 * ccs, cec24@phy.duke.edu
 * fit_planes.cpp -> extract centroids and y-axis angles
 * 
 * summary: 
 * fit planar models to segemented objects
 * if we have more than one plane, compute the centroid by finding the common
 * point +-r/2 from the centre of all the planes
 *
 * if we have one plane, then we can guess centroid but can't fix normal direction 
 * but we can still guess the angle, if its horizontal
 *
 * 
 * usage: 
 * arg1 <- (string) pcd cluster file (putative cube) 
 * arg2 <- (string) output-path, centroids+angles file gets written here, 
 * also diagnostic pcd files are spewed all over in #ifdef DEBUG
 * arg3 <- (int) cluster-index, index of cluster, used to tag the centroid + angle
 * 
 * on return:
 * the file centroids_fit.dat in output-path will have a line: index centroid.x centroid.y centroid.z angle appended to it
 * 
 */

#include "binfile.h"

std::vector<struct planeInfo> extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, std::string outpath);
float computeDistance(pcl::PointXYZ *p1, pcl::PointXYZ *p2);
pcl::PointXYZ guessCentroid(std::vector<struct planeInfo> &planesVec, int &errflag);
float computeAngle(pcl::PointXYZ centroid, std::vector<struct planeInfo> planesVec);
float computeAngleHoriz(pcl::PointXYZ centroid, std::vector<struct planeInfo> planesVec);

int main (int argc, char** argv){
	if(argc < 4) {
		std::cerr << "# fits planes to a cluster file " << std::endl;
		std::cerr << "# run with <binfile.path> <outpath> <index>" << std::endl;
		return EXIT_FAILURE;
	}
	
	std::string filepath = std::string(argv[1]); 
	std::string outpath = std::string(argv[2]); 
	int run_index = std::atoi(argv[3]);

	std::cerr << "# reading: " << filepath << std::endl;
	std::cerr << "# intermediate files output path: " << outpath << std::endl;
	std::cerr << "# index: " << run_index << std::endl;


#ifdef READCBIN
	// useful if you're suffering from os-x stringy/boost failures
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readBinfileCCS(filepath);
#else 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(filepath, *cloud);
#endif

	std::cerr << "# cloud read width: " << cloud->width;
	std::cerr << " height: " << cloud->height << std::endl;

	std::vector<struct planeInfo> planesVec;

	planesVec = extractPlanes(cloud, outpath);

	for(int i = 0; i < planesVec.size(); i++){
		std::cerr << "# " << planesVec[i].normal.x << " " << 
			planesVec[i].normal.y << " " <<
			planesVec[i].normal.z; 
		if(planesVec[i].vertical){
			std::cerr << " vertical" << std::endl;
		} else if(planesVec[i].horizontal){
			std::cerr << " horizontal" << std::endl;
		} else {
			std::cerr << " brokzen" << std::endl;
		}
	}

	if(planesVec.size() == 0){
		// didn't find any planes, compute the com and dump that
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud, centroid);
		float angle = 0.0;

		boost::filesystem::path outPathFull(outpath); // append the result string to the outpath correctly
		outPathFull /= "centroids_fit.dat";
		std::ofstream outfile;
		// open file, for output, in append mode
		outfile.open(outPathFull.c_str(), std::ios::out | std::ios::app);
		outfile << run_index << "  " << centroid[0] << " " << centroid[1] << " " << centroid[2]  << " " << angle << std::endl;
		outfile.close();
		return returnNoPlanes;
	}

	pcl::PointXYZ centroid;
	
	int errflag = 0;
	centroid = guessCentroid(planesVec, errflag);

	if(errflag == returnNANCentroid){ // bail
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud, centroid);
		float angle = 0.0;

		boost::filesystem::path outPathFull(outpath); // append the result string to the outpath correctly
		outPathFull /= "centroids_fit.dat";
		std::ofstream outfile;
		// open file, for output, in append mode
		outfile.open(outPathFull.c_str(), std::ios::out | std::ios::app);
		outfile << run_index << "  " << centroid[0] << " " << centroid[1] << " " << centroid[2]  << " " << angle << std::endl;
		outfile.close();
		return returnNANCentroid;
	}		
	
	// try and compute the angle
	float angle = 0.0;
	
	try{
		angle = computeAngle(centroid, planesVec);
	} catch (int e) {
		std::cerr << "computeAngle threw exception " << e << std::endl;
		// dump the centroid incase its any use
		std::cerr  << "centroid: " << centroid << std::endl;
		return EXIT_FAILURE;
	}


	// cout the results
	std::cout << run_index << "  " << centroid.x << " " << centroid.y << " " << centroid.z  << " " << angle << std::endl;

	// now append them to file
	boost::filesystem::path outPathFull(outpath); // append the result string to the outpath correctly
	outPathFull /= "centroids_fit.dat";
	std::ofstream outfile;
	// open file, for output, in append mode
	outfile.open(outPathFull.c_str(), std::ios::out | std::ios::app);
	outfile << run_index << "  " << centroid.x << " " << centroid.y << " " << centroid.z  << " " << angle << std::endl;
	outfile.close();
	
	// check for errors
	if(std::isnan(centroid.x) || std::isnan(centroid.y) || std::isnan(centroid.z)) {
		std::cerr << "# nan centroids\n";
		return returnNANCentroid;
	}

	if(errflag == returnGuessNormal){
		std::cerr << "# guessed normal orientation \n";
		return returnGuessNormal;
	}


	return EXIT_SUCCESS;
}


/**
 * use extracted planes to guess the centroid of the cube
 * we need > 2 planes or we can't really tell where the centroid is because we don't 
 * know the normal
 * we can just guess that the normal is outwards and then run back, but it's not obvious
 *
 * each consensus pair will sign the normals, changing planesVec. 
 *
 * i don't think its possible to have a *sane* situation where there could be multiple
 * signings of the normals, but i'm not certain yet
 */
pcl::PointXYZ guessCentroid(std::vector<struct planeInfo> &planesVec, int &errflag)
{
	int nplanes = planesVec.size();
	float guessThreshhold = 1e-2;
	float distance = 0.0;


	pcl::PointXYZ centroid;

	if(nplanes < 2){
 		/**
		 * should output some info to an error file here, incase someone wants to revist the fucked up planes
		 */
		std::cerr << "# not enough planes to be totally sure, picked positive normal" << std::endl;
		// returns  a guess, we picked the normals to be +ve (?)
		centroid.x = planesVec[0].center.x - planesVec[0].radius * planesVec[0].normal.x;
		centroid.y = planesVec[0].center.y - planesVec[0].radius * planesVec[0].normal.y;
		centroid.z = planesVec[0].center.z - planesVec[0].radius * planesVec[0].normal.z;
		// might as well give the sign, so we can guess an angle too
		planesVec[0].normalSign = 1;
		errflag = returnGuessNormal;
		return centroid;
	} 


	// for each pair of planes we want to try each combination of +- on the normals
	// if we get a pair of pts which are < guessThreshhold then we should hold that 
	// pair. Then we can compare these pts and see if they're all within each others distance
	std::vector<pcl::PointXYZ> centTryVec;

	float radiusCombinations[4][2] = {{ 1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

	pcl::PointXYZ *p1, *p2;
	// the centroid is a distance of radius back along the normal vector from the 
	// centre of the face 
	// centGuess = centre - radius * normal
	for(int i = 0; i < nplanes; i++){
		for(int j = 0; j < i; j++){
			for(int count = 0; count < 4; count++){
				p1 = new pcl::PointXYZ;
				p2 = new pcl::PointXYZ;
				// try ++, +-. -+, --
				// break if we get a winner?
				p1->x = planesVec[i].center.x - radiusCombinations[count][0] * planesVec[i].radius * planesVec[i].normal.x;
				p1->y = planesVec[i].center.y - radiusCombinations[count][0] * planesVec[i].radius * planesVec[i].normal.y;
				p1->z = planesVec[i].center.z - radiusCombinations[count][0] * planesVec[i].radius * planesVec[i].normal.z;

				p2->x = planesVec[j].center.x - radiusCombinations[count][1] *planesVec[j].radius * planesVec[j].normal.x;
				p2->y = planesVec[j].center.y - radiusCombinations[count][1] *planesVec[j].radius * planesVec[j].normal.y;
				p2->z = planesVec[j].center.z - radiusCombinations[count][1] *planesVec[j].radius * planesVec[j].normal.z;
			
				distance = computeDistance(p1, p2);
					
				#ifdef DEBUG
				// some debug blurb
				std::cerr << "# " << i << "p1 (" << p1->x << "," << p1->y << "," << p1->z << ") " << j << " p2 (" 
									<< p2->x << "," << p2->y << "," << p2->z  << ") dist: " << distance << " " << count << std::endl;
				
				#endif

				if(distance < guessThreshhold){
					// push back, p1, p2 or the average?
					centTryVec.push_back(*p1);
					// sign the normals and store them
					planesVec[i].normalSign = (float)radiusCombinations[count][0];
					planesVec[j].normalSign = (float)radiusCombinations[count][1];
					delete p1;
					delete p2;
					#ifdef DEBUG
					std::cerr << "# won count: " << count << std::endl;
					#endif
					break;
				}
				delete p1;
				delete p2;
			}
		}
	}

	centroid.x = 0; 
	centroid.y = 0;
	centroid.z = 0;


	if(centTryVec.size() == 0){
		// didn't work, just do a simple guess using PCA
		std::cerr << "# centroid guessing with planes/normals failed" << std::endl;
		errflag = returnNANCentroid;
		return centroid;
	}
		
	

	std::cerr << "# final centroid candidates:\n";
	for(int i = 0; i < centTryVec.size(); i++){ // compute the centre of mass (avg) centroid
		std::cerr << "# " << centTryVec[i] << std::endl;
		centroid.x += centTryVec[i].x;
		centroid.y += centTryVec[i].y;
		centroid.z += centTryVec[i].z;
	}
	centroid.x /= (float)centTryVec.size();
	centroid.y /= (float)centTryVec.size();
	centroid.z /= (float)centTryVec.size();

	std::cout << "# <centroid>: " << centroid << std::endl;

	#ifdef DEBUG
	std::cerr << "# normal signs: ";
	for(int i = 0; i < planesVec.size(); i++)
		std::cerr << planesVec[i].normalSign << " ";
	std::cerr << std::endl;
	#endif

	return centroid;
}

// euclidean distance between p1 and p2
float computeDistance(pcl::PointXYZ *p1, pcl::PointXYZ *p2){
	float dx, dy, dz;
	dx = p1->x - p2->x;
	dy = p1->y - p2->y;
	dz = p1->z - p2->z;
	return(sqrt(dx*dx + dy*dy + dz*dz));
}


/**
 * compute the angle in the yz plane
 * since we know the centroid we can sign the normals for all the (vertical) faces
 * the angle is atan2( normal.x, normal.z)
 *
 * if we only have horizontal faces, we have to use the edges...
 * 
 * the angle returned is in the first quadrant, theta in 0..M_PI/2 with the z axis as zero.
 */
float computeAngle(pcl::PointXYZ centroid, std::vector<struct planeInfo> planesVec){
	int nplanes = planesVec.size();
	std::vector<float> angleVec;
	const float smallAngleCutOff = 1e-1; // angles less than this will be clamped to zero
	float angleTemp = 0.0;
	float angleFinal = 0.0;
	float zTemp, xTemp;
	int meanCount = 0;

	for(int i = 0; i < nplanes; i++){
		if(planesVec[i].vertical){
			zTemp = planesVec[i].normal.z * planesVec[i].normalSign;
			xTemp = planesVec[i].normal.x * planesVec[i].normalSign;
			//angleTemp = atan2(zTemp, xTemp);
			angleTemp = atan(xTemp/ zTemp);

			if(std::fabs(angleTemp) < smallAngleCutOff || std::fabs(M_PI/2 - angleTemp) < smallAngleCutOff ){ // clamp small values to zero
				angleTemp = 0.0;
			}

			angleVec.push_back(angleTemp);
		} else if(planesVec[i].horizontal){
			// try to do a horiz angle
			angleVec.push_back(computeAngleHoriz(centroid, planesVec));
		}
	}

	if(angleVec.size() == 0){
		return(computeAngleHoriz(centroid, planesVec));
	}

	// return the average?
	angleTemp = 0.0;

#ifdef DEBUG
	std::cerr << "# angles:\n";
	#endif
	for(int i = 0; i < angleVec.size(); i++){
		#ifdef DEBUG
		std::cerr << "# angleVec: " << angleVec[i] << std::endl;
		#endif

		angleTemp = angleVec[i];
		if(!std::isnan(angleTemp)){
			if(angleTemp < 0){
				angleTemp = 2 * M_PI + angleTemp;
			} 

		#ifdef DEBUG
			std::cerr << "# angleTemp: (preMod) " << angleTemp << std::endl;
		#endif
			angleTemp = fmod(angleTemp, M_PI/2);
		#ifdef DEBUG
			std::cerr << "# angleTemp: " << angleTemp << std::endl;
		#endif

		// we'll just look at the angle mod PI
			angleFinal += angleTemp;
			meanCount++;
		}
	}

	std::cerr << "# angle final: " << angleFinal << " meanCount: " <<  meanCount << std::endl;

	angleFinal /= (double)(meanCount);

	std::cerr << "# angle final: " << angleFinal << std::endl;
	
	return(angleFinal);
}

/**
 * try and use the edges of the horizontal faces to extract an angle
 *
 * small angles are clamped to zero, this stops problems with averaging 
 * say: 0, 2*PI - epsilon, epsilon, 0, which gives you 2PI/4 instead of ~ 0
 * 
 */
float computeAngleHoriz(pcl::PointXYZ centroid, std::vector<struct planeInfo> planesVec){
	int nplanes = planesVec.size();
	std::vector<float> angleVec;
	const float smallAngleCutOff = 1e-1; // angles less than this will be clamped to zero
	float angleTemp = 0.0;
	float angleFinal = 0.0;
	float z1, x1;
	float z2, x2;
	int meanCount = 0;

	for(int i = 0; i < nplanes; i++){
		if(planesVec[i].horizontal){

			z1 = planesVec[i].cloud_hull[0].z;
			x1 = planesVec[i].cloud_hull[0].x;

			z2 = planesVec[i].cloud_hull[1].z;
			x2 = planesVec[i].cloud_hull[1].x;

			#ifdef DEBUG
			std::cerr << "p1: " << x1 << ", " << z1  << std::endl;
			std::cerr << "p2: " << x2  << ", " << z2 <<  std::endl;
			#endif
			
			angleTemp =  atan( (x1-x2) / (z1-z2) );
			if(std::fabs(angleTemp) < smallAngleCutOff || std::fabs(M_PI/2 - angleTemp) < smallAngleCutOff){ // clamp very small angles to zero
				angleTemp = 0.0;
			} 
			angleVec.push_back(angleTemp);
		}
	}
	
	if(angleVec.size() == 0){
		return 0;
	}
	
	for(int i = 0; i < angleVec.size(); i++){
		angleTemp = angleVec[i];
 
		#ifdef DEBUG
		std::cerr << "# " << angleTemp  << std::endl;
		#endif
		
		if(angleTemp < -0.01){
			angleTemp = 2*M_PI + angleTemp;
		}
		// we'll just look at the angle mod PI
		angleFinal += (fmod(angleTemp, M_PI/2));
		meanCount++;
	}

	angleFinal /= (double)(meanCount);

	#ifdef DEBUG
	std::cerr << "# angle final: " << angleFinal << std::endl;
	#endif
	
	return(angleFinal);
}


/**
 * try and extract planes from the cloud, this should represent
 * a segmented object
 */
std::vector<struct planeInfo> extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, std::string outpath){
	float distThreshGuess = 0.001; 
	float cutOffTiny = 9E-3; // how small a dx is small enough to ignore?
	// how much do the normals have to project into either the vertical or horizonal 
	// directions to count this as a vertical or a horizontal plane?
	const float projCutOff = 0.9;
	// none of the range (min,max) values should be bigger than this
	const float saneRangeMax = 5.0; 
	int sizeMin = 0;
	int inlierCutOff = 40;
																	 
	struct planeInfo *tempPlane;
	std::vector<struct planeInfo> planeInfoVec;
	planeInfoVec.reserve(6);
		
	std::stringstream ss;
	boost::filesystem::path outPathFull(outpath); // append the result string to the outpath correctly


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlane (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWorking (new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjected (new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);	

	cloudWorking = cloudPtr; // make a copy of the input cloud
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distThreshGuess); // try and force the extraction of a single planar cube, not a whole fuckton of them
	seg.setRadiusLimits(cubeShortSide, cubeLongSide);

	// the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
	
	float bigValue = 1E6;
	
	// std::vector<pcl::PointXYZ> normVec;
	// std::vector<pcl::PointXYZ> centVec;
	//pcl::PointXYZ *p1;

	int i = 0; //nr_points = (int) cloudWorking->points.size ();
  // While 10% of the original cloud is still there
  while (cloudWorking->points.size () > sizeMin)
  {
    // Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloudWorking);
		seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () < inlierCutOff)
    {
      std::cerr << "# Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

		std::cerr << "# Model coefficients: " << coefficients->values[0] << " " 
							<< coefficients->values[1] << " "
							<< coefficients->values[2] << " " 
							<< coefficients->values[3] << std::endl;
		
		tempPlane = new struct planeInfo;
		tempPlane->horizontal = false;
		tempPlane->vertical = false;
		tempPlane->normalSign = 0.0;
		tempPlane->coeffs[0] = coefficients->values[0];
		tempPlane->coeffs[1] = coefficients->values[1];
		tempPlane->coeffs[2] = coefficients->values[2];
		tempPlane->coeffs[3] = coefficients->values[3];
		

		// store the normal to this plane, this is the coeffs a,b,c
		// the coeff 4 is the distance of the centre of the plane from the origin?
		tempPlane->normal.x = coefficients->values[0];
		tempPlane->normal.y = coefficients->values[1];
		tempPlane->normal.z = coefficients->values[2];

    // Extract the inliers
    extract.setInputCloud (cloudWorking);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloudPlane);

    std::cerr << "# planar PointCloud: " << cloudPlane->width * cloudPlane->height << " data points." << std::endl;

		#ifdef DEBUGPROJ
		// before projection
		std::cerr << "# Cloud before projection: " << std::endl;
		for (size_t index = 0; index < 10; ++index)
    std::cerr << "    " << cloudPlane->points[index].x << " " 
                        << cloudPlane->points[index].y << " " 
                        << cloudPlane->points[index].z << std::endl;
		#endif

		// so we project the inliers into the model, this keeps the convex hull planes flat
		// space or something else?
		pcl::ProjectInliers<pcl::PointXYZ> proj;
		proj.setModelType (pcl::SACMODEL_PLANE);
		proj.setInputCloud (cloudPlane);
		proj.setModelCoefficients (coefficients);
		proj.filter (*cloudProjected);

    #ifdef DEBUGPROJ
		std::cerr << "# PointCloud after projection has: "
							<< cloudProjected->points.size () << " data points." << std::endl;
		
		// for each plane we want to find the extents in x, y, z
		std::cerr << "Cloud after projection: " << std::endl;
		for (size_t index = 0; index < 10; ++index)
    std::cerr << "    " << cloudProjected->points[index].x << " " 
                        << cloudProjected->points[index].y << " " 
                        << cloudProjected->points[index].z << std::endl;
		#endif

		
		// Create a Convex Hull representation of the projected inliers
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ConvexHull<pcl::PointXYZ> chull;

		chull.setInputCloud (cloudProjected);
		//chull.setComputeAreaVolume(true);

		chull.reconstruct (*cloud_hull);
		
		std::cerr << "# chull dim: " << chull.getDim();
		std::cerr << " npts: " << cloud_hull->points.size() << std::endl;

		float xmin = bigValue, xmax = -bigValue, ymin = bigValue, ymax= -bigValue, zmin = bigValue , zmax = -bigValue;

		pcl::PointXYZ minPt, maxPt;
		pcl::getMinMax3D(*cloud_hull, minPt, maxPt);
		xmin = minPt.x;
		xmax = maxPt.x;
		ymin = minPt.y;
		ymax = maxPt.y;
		zmin = minPt.z;
		zmax = maxPt.z;

		tempPlane->cloud_hull = *cloud_hull;

		#ifdef DEBUG
		std::cerr << "# xrange: " << xmin << " " << xmax << std::endl;
		std::cerr << "# yrange: " << ymin << " " << ymax << std::endl;
		std::cerr << "# zrange: " << zmin << " " << zmax << std::endl;
		#endif
		
		double dx, dy, dz, centX, centY, centZ;
		dx = fabs(xmax-xmin);
		dy = fabs(ymax-ymin);
		dz = fabs(zmax-zmin);

		#ifdef DEBUG
		std::cerr << "# dx: " << dx << std::endl;
		std::cerr << "# dy: " << dy << std::endl;
		std::cerr << "# dz: " << dz << std::endl;
		#endif
		
		tempPlane->xrange[0] = xmin;
		tempPlane->xrange[1] = xmax;
		tempPlane->yrange[0] = ymin;
		tempPlane->yrange[1] = ymax;
		tempPlane->zrange[0] = zmin;
		tempPlane->zrange[1] = zmax;

		// find the centre of this plane
		centX = xmin + dx/2;
		centY = ymin + dy/2;
		centZ = zmin + dz/2;

		tempPlane->center.x = centX;
		tempPlane->center.y = centY;
		tempPlane->center.z = centZ;

		#ifdef DEBUG
		std::cerr << "# centX: " << centX << std::endl;
		std::cerr << "# centY: " << centY << std::endl;
		std::cerr << "# centZ: " << centZ << std::endl;
		#endif

		float projVertNorm = 0.0;
		float projHorizNorm = 0.0;
		
		// note: we don't know sign of normals, so we project with |n| 
		// how much is projected into the vertical direction 
		projVertNorm = std::fabs(tempPlane->normal.x) + std::fabs(tempPlane->normal.z); // use fabs to make sure both norms contrib
		// how much is projected into the horizonal plane
		projHorizNorm = tempPlane->normal.y;
		
		#ifdef DEBUG
		std::cerr << "# projVertNorm: " << projVertNorm; 
		std::cerr << " projHorizNorm: " << projHorizNorm << std::endl;
		#endif DEBUG

		if(std::fabs(projVertNorm) >= projCutOff && std::fabs(projHorizNorm) < projCutOff){
			tempPlane->vertical = true;
			tempPlane->horizontal = false;
			tempPlane->radius = cubeShortSide / 2.0;
		} else if(std::fabs(projVertNorm) < projCutOff && std::fabs(projHorizNorm) >= projCutOff){
			tempPlane->vertical = false;
			tempPlane->horizontal = true;
			tempPlane->radius = cubeLongSide / 2.0;
		} else {
			tempPlane->horizontal = false;
			tempPlane->vertical = false;
			tempPlane->radius = 0.0;
		}

		planeInfoVec.push_back(*tempPlane);
		delete tempPlane;

		// save the planes to file, only in debug mode
#ifdef DEBUG
#ifdef WRITECBIN
		// do the boost song and dance to write the data nicely
		ss << "cloud-plane-hull-extract-" << i << ".cbin" ;
		outPathFull.clear();
		outPathFull /= outpath;
		outPathFull /= ss.str();
		writeBinfileCCS(cloud_hull, outPathFull.native());
		ss.clear();
		ss.str("");
		ss << "cloud-plane-extract-" << i << ".cbin";
		outPathFull.clear();
		outPathFull /= outpath;
		outPathFull /= ss.str();
		writeBinfileCCS(cloudPlane, outPathFull.native());
#else
		ss << "cloud-plane-hull-extract-" << i << ".pcd" ;
		outPathFull.clear();
		outPathFull /= outpath;
		outPathFull /= ss.str();
		std::cerr << "# writing file to: " << outPathFull.native() << std::endl;		
		pcl::io::savePCDFileASCII(outPathFull.native(), *cloud_hull);

		ss.clear();
		ss.str("");
		ss << "cloud-plane-extract-" << i << ".pcd" ;
		outPathFull.clear();
		outPathFull /= outpath;
		outPathFull /= ss.str();
		std::cerr << "# writing file to: " << outPathFull.native() << std::endl;		
		pcl::io::savePCDFileASCII(outPathFull.native(), *cloudPlane);

		// reset the strings for the next plane
		ss.clear();
		ss.str("");
		outPathFull.clear();
#endif
#endif

    // Create the filtering object again
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloudWorking = cloud_f;
    i++;
  }

	// now we should have found normVec.size() planes. 
	// we need to try and figure out the appropriate normal orientation
	int nplanes = planeInfoVec.size();
	std::cout << "# nplanes: " << nplanes << std::endl;

	// finally, we go through and check dims of each vertical plane, if there's one thats
	// very small in the y direction we'll assign its y center to that of its brothers that are ok
	// 
	// this won't work when ALL the planes are partial, so yeah should check for that
	double dy;
	int *badCents = new int[nplanes]; // (if 0 then this plane has a good cent, otherwise -1)
	float goodCentYV = 0.0;
	int goodCentCount = 0;
	float badLengthPercent = 0.2;
	for(int i = 0; i < nplanes; i++){
		if(planeInfoVec[i].vertical){
			dy = planeInfoVec[i].yrange[1] - planeInfoVec[i].yrange[0];
			if(fabs(dy - cubeLongSide)/cubeLongSide > badLengthPercent){ // more than 10% off but it IS a vertical side
				badCents[i] = 1;
				std::cerr << "# bad cent index: " << i << " dy: " << dy << std::endl;
			} else {
				badCents[i] = 0;
				goodCentYV += planeInfoVec[i].center.y;
				goodCentCount++;
			}
		}
	}

	// if we actually found one that's ok
	if(goodCentCount > 0){
		goodCentYV /= (float)(goodCentCount);
		std::cerr << "# good cent value: " << goodCentYV << std::endl;
	// 2nd pass, now correct the planes with the bad Y-cents
		for(int i = 0; i < nplanes; i++){
			if(badCents[i] == 1){
				planeInfoVec[i].center.y = goodCentYV;
			}
		}
	} else {
		std::cerr << "# no good partial planes" << std::endl;
	}
		

	delete [] badCents;
	
	return planeInfoVec;
}

