

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
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include "binfile.h"


int main (int argc, char** argv)
{
	// int width, height;
	// int nPoints;
	// float *pointsX, *pointsY, *pointsZ;
	// float px, py, pz;
	
	if(argc < 2){
		std::cerr << "# run with: <segfile.pcd> <outpath>" << std::endl;
		return EXIT_FAILURE;
	}

	std::string filename = std::string(argv[1]);
	std::string outpath = std::string(argv[2]);
	
	std::cerr << "# reading: " << filename << std::endl;
	std::cerr << "# output to: " << outpath << std::endl;

  // Read in the cloud data
	// the reader object seems to throw std::bad_cast errors which 
	// perhaps come from boost, lets avoid this 
  //pcl::PCDReader reader;
	// 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// io::loadPCDFile seems to be immune to the bad cast errors

	//pcl::PointCloud<pcl::PointXYZ> cloudIn;
	pcl::io::loadPCDFile(filename, *cloud);

  std::cerr << "# PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

	// // ccs, do we need to filter the data, not sure what the downsampling is doing yet

  // // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.001f, 0.001f, 0.001f);
  vg.filter (*cloud_filtered);
  std::cerr << "# PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*


  // // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);



	std::cerr << "# carrying out cluster extraction " << std::endl;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	
	/**
	 * these values need to be tuned i fear
	 * the cluster tolerance should be ~ largest diagonal distance in the cube
	 * it should be << E[distance_between_cubes] or we'll merge pairs of separate
	 * objects
	 * 
	 * 
	 * minClusterSize && maxCluster size do exactly what they say, if
	 * a cube is 2x2x4 and the size of a pixel is pixSize then 
	 * npixCubeMax = SurfaceArea / pixSize;
	 * 
	 * this should be set using the camera resolution and the approximate distance of the 
	 * camera from the object
	 * 
	 */
	float cubeShortSide = 0.02;
	float cubeLongSize = 0.04;
	float cubeSurfaceArea = 2*cubeShortSide*cubeShortSide + 4*cubeLongSize * cubeLongSize;
	float pixArea = 0.001;
	int maxCubeCluster = 2000; //cubeSurfaceArea / pixSize; (set by hand right now)
	
  ec.setClusterTolerance (0.01); // 1cm seems to work ok (this is slightly a magic param) ?
  ec.setMinClusterSize (50); // 10 is probably too low
  ec.setMaxClusterSize (maxCubeCluster); // this may be too high
  ec.setSearchMethod (tree);
  ec.setInputCloud( cloud_filtered);
  ec.extract (cluster_indices);

	std::cerr << "# nclusters:  " << cluster_indices.size() << std::endl;

  //pcl::PCDWriter writer;

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);


    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << outpath << "/cloud_cluster_" << j << ".cbin";
		//writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		//pcl::io::savePCDFileASCII(ss.str(), *cloud_cluster);
		writeBinfileCCS(cloud_cluster, ss.str());
		
    j++;
		if(j > 2){
			exit(1);
		}
  }

  return (0);
}
