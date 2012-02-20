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

#include <pcl/surface/concave_hull.h>

#include <boost/filesystem.hpp>

/**
 * fit planar models to segemented objects
 * if we have more than one plane, compute the centroid by finding the common
 * point +-r/2 from the centre of all the planes
 *
 * if we have one plane, then we can guess centroid but can't fix normal direction 
 * absolutely
 *
 * \bug segfaulting currently?
 */

// holds info about a plane,
// particularly: the centre, the x,y,z ranges, if its a vertical (long side) or horizontal plane
// estimated normals
struct planeInfo{
	/** 
	 * the estimated centre of the plane */
	pcl::PointXYZ center;
	/**
	 * the estimated normal of the plane 
	 * the sign is initially not known */
	pcl::PointXYZ normal; 
	/**
	 * the ranges for the x y z extents */
	double xrange[2];
	double yrange[2];
	double zrange[2]; 
	/**
   * is this a long edge plane in the xy  
	 * dims would be 6 x 2 */
	bool vertical; 
	/**
	 * is this an end cap, smaller 
	 * dims are: 2 x 2 */
	bool horizontal;
	/**
	 * coeffs of the ransac model that produced this */
	double coeffs[4];
	/**
	 * the distance of this surface from the centroid (supossedly) */
	double radius;
	
};

#include "binfile.h"

std::vector<struct planeInfo> extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, std::string outpath);
float computeDistance(pcl::PointXYZ *p1, pcl::PointXYZ *p2);
pcl::PointXYZ guessCentroid(std::vector<struct planeInfo> planeVec);


int main (int argc, char** argv){
	float cubeShortSide = 0.02;
	float cubeLongSide = 0.06; // seems to be 0.06 not 0.04?
	
	if(argc < 2) {
		std::cerr << "# fits planes to a cluster file " << std::endl;
		std::cerr << "# run with <binfile.path> <outpath>" << std::endl;
	}
	
	std::string filepath = std::string(argv[1]); 
	std::string outpath = std::string(argv[2]); 
	std::cerr << "# processing: " << filepath << std::endl;
	std::cerr << "# outputto: " << outpath << std::endl;

#ifdef READCBIN
	// useful if you're suffering from os-x stringy/boost failures
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readBinfileCCS(filepath);
#else 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(filepath, *cloud);
#endif

	std::cerr << "# read width: " << cloud->width << std::endl;
	std::cerr << "# read height: " << cloud->height << std::endl;

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
	
	guessCentroid(planesVec);

	

	return EXIT_SUCCESS;
}


/**
 * use extracted planes to guess the centroid of the cube
 * we need > 2 planes or we can't really tell where the centroid is because we don't 
 * know the normal
 * we can just guess that the normal is outwards and then run back, but it's not obvious
 */
pcl::PointXYZ guessCentroid(std::vector<struct planeInfo> planesVec){
	int nplanes = planesVec.size();
	float guessThreshhold = 1e-3;
	float distance = 0.0;


	pcl::PointXYZ centroid;

	if(nplanes < 2){
		std::cerr << "# not enough planes to be totally sure" << std::endl;
		// returns  a guess
		centroid.x = planesVec[0].center.x - planesVec[0].radius * planesVec[0].normal.x;
		centroid.y = planesVec[0].center.y - planesVec[0].radius * planesVec[0].normal.y;
		centroid.z = planesVec[0].center.z - planesVec[0].radius * planesVec[0].normal.z;
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
					
				// some debug blurb
				std::cerr << "# p1 (" << p1->x << "," << p1->y << "," << p1->z << ") p2 (" 
									<< p2->x << "," << p2->y << "," << p2->z  << ") dist: " << distance << std::endl;

				if(distance < guessThreshhold){
					// push back, p1, p2 or the average?
					centTryVec.push_back(*p1);
					delete p1;
					delete p2;
					std::cerr << "# won count: " << count << std::endl;
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

	std::cerr < "# final centroid candidates:\n";
	for(int i = 0; i < centTryVec.size(); i++){ // compute the centre of mass (avg) centroid
		std::cerr << "# " << centTryVec[i] << std::endl;
		centroid.x += centTryVec[i].x;
		centroid.y += centTryVec[i].y;
		centroid.z += centTryVec[i].z;
	}
	centroid.x /= (float)centTryVec.size();
	centroid.y /= (float)centTryVec.size();
	centroid.z /= (float)centTryVec.size();

	std::cout << "# centroid: " << centroid << std::endl;

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
 * the angle is atan2( normal.y, normal.z)
 */
float computeAngle(pcl::PointCloud<pcl::PointXYZ> centroid, std::vector<struct planeInfo> planesVec){

}


/**
 * try and extract planes from the cloud, this should represent
 * a segmented object
 */
std::vector<struct planeInfo> extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, std::string outpath){
	float cubeShortSide = 0.02;
	float cubeLongSide = 0.06; // seems to be 0.06 not 0.04?
	float distThreshGuess = 0.001; 
	float cutOffTiny = 1E-3; // how small a dx is small enough to ignore?

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
	
	double bigValue = 1E10;
	
	// std::vector<pcl::PointXYZ> normVec;
	// std::vector<pcl::PointXYZ> centVec;
	//pcl::PointXYZ *p1;

	int i = 0, nr_points = (int) cloudWorking->points.size ();
  // While 10% of the original cloud is still there
  while (cloudWorking->points.size () > 0.1 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloudWorking);
		seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

		std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
							<< coefficients->values[1] << " "
							<< coefficients->values[2] << " " 
							<< coefficients->values[3] << std::endl;
		
		tempPlane = new struct planeInfo;
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

    std::cerr << "# PointCloud representing the planar component: " << cloudPlane->width * cloudPlane->height << " data points." << std::endl;

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
		std::cerr << "# PointCloud after projection has: "
							<< cloudProjected->points.size () << " data points." << std::endl;

		#ifdef DEBUGPROJ
		// for each plane we want to find the extents in x, y, z
		std::cerr << "Cloud after projection: " << std::endl;
		for (size_t index = 0; index < 10; ++index)
    std::cerr << "    " << cloudProjected->points[index].x << " " 
                        << cloudProjected->points[index].y << " " 
                        << cloudProjected->points[index].z << std::endl;
		#endif

		
		// how can we do this?
		// 
		// Create a Convex Hull representation of the projected inliers
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ConvexHull<pcl::PointXYZ> chull;

		chull.setInputCloud (cloudProjected);
		//chull.setComputeAreaVolume(true);

		std::cerr << "# chull reconstruction starting\n";

		chull.reconstruct (*cloud_hull);

		std::cerr << "# chull reconstruction done\n";
		
		std::cerr << "# chull dim: " << chull.getDim() << std::endl;
		std::cerr << "# chull npts: " << cloud_hull->points.size() << std::endl;

		double xmin = bigValue, xmax = -bigValue, ymin = bigValue, ymax= -bigValue, zmin = bigValue , zmax = -bigValue;

		// do a stupid search in the convex hull, this is not going to be many points
		// so we can extract min and max values (i hope)
		for(int index = 0; index < cloud_hull->points.size(); index++){
			if(cloud_hull->points[index].x > xmax){
				xmax = cloud_hull->points[index].x;
			} else if (cloud_hull->points[index].x < xmin){
				xmin = cloud_hull->points[index].x;
			}
			if(cloud_hull->points[index].y > ymax){
				ymax = cloud_hull->points[index].y;
			} else if (cloud_hull->points[index].y < ymin){
				ymin = cloud_hull->points[index].y;
			}
			if(cloud_hull->points[index].z > zmax){
				zmax = cloud_hull->points[index].z;
			} else if (cloud_hull->points[index].z < zmin){
				zmin = cloud_hull->points[index].z;
			}

		}

		std::cerr << "xrange: " << xmin << " " << xmax << std::endl;
		std::cerr << "yrange: " << ymin << " " << ymax << std::endl;
		std::cerr << "zrange: " << zmin << " " << zmax << std::endl;
		
		double dx, dy, dz, centX, centY, centZ;
		dx = fabs(xmax-xmin);
		dy = fabs(ymax-ymin);
		dz = fabs(zmax-zmin);

		std::cerr << "dx: " << dx << std::endl;
		std::cerr << "dy: " << dy << std::endl;
		std::cerr << "dz: " << dz << std::endl;
		
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

		std::cerr << "centX: " << centX << std::endl;
		std::cerr << "centY: " << centY << std::endl;
		std::cerr << "centZ: " << centZ << std::endl;
		
		// finally we should determine if its a vertical or horizontal plane
		// if its vertical then the area should be 2 x 6, and the face will be in either the
		// xy || zy planes the radius is cubeShortSide / 2
		// if its a horizontal plane then it has to be in the xz plane
		// then the radius is cubeLongSide /2 
		if(dx > cutOffTiny && dy > cutOffTiny && dz <= cutOffTiny || dz > cutOffTiny && dy > cutOffTiny && dx <= cutOffTiny){
			tempPlane->horizontal = false; 
			tempPlane->vertical = true;
			tempPlane->radius = cubeShortSide / 2.0;
		} else if( dx > cutOffTiny && dz > cutOffTiny && dy <= cutOffTiny ){
			tempPlane->horizontal = true;
			tempPlane->vertical = false;
			tempPlane->radius = cubeLongSide / 2.0;
		} else {
			tempPlane->horizontal = false;
			tempPlane->vertical = false;
			tempPlane->radius = 0.0;
		}
	

		planeInfoVec.push_back(*tempPlane);
		delete tempPlane;


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
	
	return planeInfoVec;
}

