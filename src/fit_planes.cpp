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
 */


#include "binfile.h"

void extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, std::string outpath);

int main (int argc, char** argv){
	
	if(argc < 2) {
		std::cerr << "# fits planes to a cluster file " << std::endl;
		std::cerr << "# run with <binfile.path> <outpath>" << std::endl;
	}
	
	std::string filepath = std::string(argv[1]); 
	std::string outpath = std::string(argv[2]); 
	std::cerr << "# processing: " << filepath << std::endl;
	std::cerr << "# outputto: " << outpath << std::endl;

#ifdef READCBIN
	// useful if you're suffering from the boost failures
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readBinfileCCS(filepath);
#else 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(filepath, *cloud);
#endif

	std::cerr << "# read width: " << cloud->width << std::endl;
	std::cerr << "# read height: " << cloud->height << std::endl;

	extractPlanes(cloud, outpath);

	return EXIT_SUCCESS;
}



/**
 * try and extract planes from the cloud, this should represent
 * a segmented object
 */
void extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, std::string outpath){
	float cubeShortSide = 0.02;
	float cubeLongSide = 0.06; // seems to be 0.06 not 0.04?
	float distThreshGuess = 0.001; 
	bool longSideFlag = false;
	bool shortSideFlag= false;
	bool workedFlag = false;
		
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
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;


	double bigValue = 1E10;
	
	std::vector<pcl::PointXYZ> normVec;
	std::vector<pcl::PointXYZ> centVec;
	pcl::PointXYZ *p1;

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
		
		// store the normal to this plane, this is the coeffs a,b,c
		// the coeff 4 is the distance of the centre of the plane from the origin?
		p1 = new pcl::PointXYZ;
		p1->x = coefficients->values[0];
		p1->y = coefficients->values[1];
		p1->z = coefficients->values[2];
		normVec.push_back(*p1);
		delete p1;

    // Extract the inliers
    extract.setInputCloud (cloudWorking);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloudPlane);
    std::cerr << "PointCloud representing the planar component: " << cloudPlane->width * cloudPlane->height << " data points." << std::endl;


		#ifdef DEBUG
		// before projection
		std::cerr << "Cloud before projection: " << std::endl;
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
		std::cerr << "PointCloud after projection has: "
							<< cloudProjected->points.size () << " data points." << std::endl;

		#ifdef DEBUG
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
		chull.setInputCloud (cloudProjected);
		chull.setComputeAreaVolume(true);

		std::cerr << "# chull reconstruction starting\n";

		chull.reconstruct (*cloud_hull);

		std::cerr << "# chull reconstruction done\n";
		
		std::cerr << "# chull dim: " << chull.getDim() << std::endl;
		std::cerr << "# chull npts: " << cloud_hull->points.size() << std::endl;

		double xmin = bigValue, xmax = -bigValue, ymin = bigValue, ymax= -bigValue, zmin = bigValue , zmax = -bigValue;

		// this is wrong so far, should be measuring distance along the plane coords not 
		// distance in the global coords
		// 
		// so we now do a stupid search in the convex hull, this is not going to be many points
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
		
		// find the centre of this plane
		centX = xmin + dx/2;
		centY = ymin + dy/2;
		centZ = zmin + dy/2;

		p1 = new pcl::PointXYZ;
		p1->x = centX;
		p1->y = centY;
		p1->z = centZ;
		centVec.push_back(*p1);
		delete p1;

		std::cerr << "centX: " << centX << std::endl;
		std::cerr << "centY: " << centY << std::endl;
		std::cerr << "centZ: " << centZ << std::endl;


#ifdef WRITECBIN
		// do the boost song and dance to write the data nicely
		ss << "cloud-plane-hull-extract-" << i << ".cbin" ;
		outPathFull.clear();
		outPathFull /= outpath;
		outPathFull /= ss.str();;
		writeBinfileCCS(cloud_hull, outPathFull.native());
		ss.clear();
		ss.str("");
		ss << outpath << "/cloud-plane-extract-" << i << ".cbin" ;
		writeBinfileCCS(cloudPlane, ss.str());
#else
		ss << "cloud-plane-hull-extract-" << i << ".pcd" ;
		outPathFull.clear();
		outPathFull /= outpath;
		outPathFull /= ss.str();;
		pcl::io::savePCDFileASCII(outPathFull.native(), *cloud_hull);
		ss.clear();
		ss.str("");
		ss << outpath << "/cloud-plane-extract-" << i << ".cbin" ;
		pcl::io::savePCDFileASCII(outPathFull.native(), *cloudPlane);
#endif
		

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloudWorking = cloud_f;
    i++;
  }

	// now we should have found normVec.size() planes. 
	// we need to try and figure out the appropriate normal orientation

}

