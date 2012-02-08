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

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readBinfileCCS(filepath);

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

	int i = 0, nr_points = (int) cloudWorking->points.size ();
  // While 30% of the original cloud is still there
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


    // Extract the inliers
    extract.setInputCloud (cloudWorking);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloudPlane);
    std::cerr << "PointCloud representing the planar component: " << cloudPlane->width * cloudPlane->height << " data points." << std::endl;


		pcl::ProjectInliers<pcl::PointXYZ> proj;
		proj.setModelType (pcl::SACMODEL_PLANE);
		proj.setInputCloud (cloudPlane);
		proj.setModelCoefficients (coefficients);
		proj.filter (*cloudProjected);
		std::cerr << "PointCloud after projection has: "
							<< cloudProjected->points.size () << " data points." << std::endl;
		// for each plane we want to find the extents in x, y, z
		// how can we do this?
		// 
		// Create a Convex Hull representation of the projected inliers
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ConvexHull<pcl::PointXYZ> chull;
		chull.setInputCloud (cloudProjected);
		//chull.setComputeAreaVolume(true);
		chull.reconstruct (*cloud_hull);
		
		std::cerr << "# chull dim: " << chull.getDim() << std::endl;
		//std::cerr << "# chull area: " << chull.getTotalArea() << std::endl;
		std::cerr << "# chull npts: " << cloud_hull->points.size() << std::endl;
		double xmin = bigValue, xmax = -bigValue, ymin = bigValue, ymax= -bigValue, zmin = bigValue , zmax = -bigValue;
		// std::cerr << xmin << " " << xmax << std::endl;
		// std::cerr << ymin << " " << ymax << std::endl;
		// std::cerr << zmin << " " << zmax << std::endl;

		// need to project points into the model space first then do this
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
		
		double dx, dy, dz;
		dx = fabs(xmax-xmin);
		dy = fabs(ymax-ymin);
		dz = fabs(zmax-zmin);

		std::cerr << "dx: " << dx << std::endl;
		std::cerr << "dy: " << dy << std::endl;
		std::cerr << "dz: " << dz << std::endl;
		
		// now we can do something smart with the sides?
		// we have the normals from the model coeffs and we have the 
		// extents of that side, so we know roughly the orientation
		// of the cube and its centroid. Right?
		// 
		// 1) if we have 3 (roughly perpendicular) normals then we know that their intersection 
		// defines the centre of the cube, also they define the rotation of the cube
		//
		// 2) if we have 2 normals we can reconstruct the cube from knowing if they 
		// correspond to short (end caps) or long faces (side panels), right?
		// 
		//
		// 3) if we have 1 normal  (can't really do anything)
		
		if( fabs(dz - cubeShortSide) > 1e-2){
			std::cerr << "found a long side" << std::endl;
			longSideFlag = true;
			workedFlag = true;
		} else if(fabs(dx-cubeShortSide) < 1e-2 && fabs(dz - cubeLongSide) < 1e-2){
			std::cerr << "found a short side" << std::endl;
			shortSideFlag = true;
			workedFlag = true; 
		} else {
			std:: cerr << "found a who knows what :(" << std::endl;
			workedFlag = false;
		}
			

    std::stringstream ss1;
		ss1 << outpath << "/cloud-plane-hull-extract-" << i << ".cbin" ;
		writeBinfileCCS(cloud_hull, ss1.str());
		

    std::stringstream ss;
		ss << outpath << "/cloud-plane-extract-" << i << ".cbin" ;
		writeBinfileCCS(cloudPlane, ss.str());

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloudWorking = cloud_f;
    i++;
  }
}

