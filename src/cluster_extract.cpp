

#include "stdio.h"
#include "constants.h"

#include <iostream>
#include <fstream>
#include <locale>
#include <string>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
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

#include <boost/filesystem.hpp>

/** 
 * ccs, cec24@phy.duke.edu
 * read an input pcd file, filter it and then 
 * extract euclidean clusters which are 
 * output as separate pcd files into outpath
 * 
 * variables to be tuned: 
 * minClusterSize && maxClustersize 
 * 
 * may not want to do the filtering, this seems to squash the outliers
 */

int main (int argc, char** argv)
{
	
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;// (new pcl::PointCloud<pcl::PointXYZ>);

	bool filterData = false;

	if(filterData){
  std::cerr << "# PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
	// // ccs, do we need to filter the data, not sure what the downsampling is doing yet
  // Create the filtering object: downsample the dataset using a leaf size of 1mm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.001f, 0.001f, 0.001f);
  vg.filter (*cloud_filtered);
  std::cerr << "# PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
	} else{
		// copy?
		cloud_filtered = cloud;
	}

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
	//float cubeSurfaceArea = 2*cubeShortSide*cubeShortSide + 4*cubeLongSize * cubeLongSize;
	//float pixArea = 0.001;
	int maxCubeCluster = 2000; //cubeSurfaceArea / pixSize; (set by hand right now)
	
  ec.setClusterTolerance (0.01); // 1cm seems to work ok (this is slightly a magic param) ?
	// seems like < 400 is too small for icp to work?
  ec.setMinClusterSize (50); // 10 is probably too low
  ec.setMaxClusterSize (maxCubeCluster); // this may be too high
  ec.setSearchMethod (tree);
  ec.setInputCloud( cloud_filtered);
  ec.extract (cluster_indices);

	std::cerr << "# nclusters:  " << cluster_indices.size() << std::endl;


	std::stringstream ss;
	boost::filesystem::path outPathFull(outpath); // append the result string to the outpath correctly

	// when you think like a hermit you forget what you know
	Eigen::Vector4f centroidVec;

	std::ofstream centroidGuessFile;
	ss << "centroid_pca_guess.txt";
	outPathFull /= ss.str();
	centroidGuessFile.open(outPathFull.c_str());

	outPathFull.clear();
	ss.clear();
	ss.str("");

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);


		// do a hard copy over the indices
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

		// this should presumably go to a file too
		pcl::compute3DCentroid(*cloud_cluster, centroidVec);
		std::cout << "# guess centroid: " << centroidVec << std::endl;
		centroidGuessFile << j << " " << centroidVec.x() << " " << centroidVec.y() << " " << centroidVec.z() << std::endl;

    //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    ss << "cloud_cluster_" << j << ".pcd";
		outPathFull.clear();
		outPathFull /= outpath;
		outPathFull /= ss.str(); // add on the string stream part
		pcl::io::savePCDFileASCII(outPathFull.native(), *cloud_cluster);
		ss.clear();
		ss.str("");


		#ifdef WRITECBIN
		ss << "cloud_cluster_" << j << ".cbin";
		outPathFull.clear();
		outPathFull /= outpath;
		outPathFull /= ss.str(); // add on the string stream part
		writeBinfileCCS(cloud_cluster, outPathFull.native());
		#endif
    j++;
  }

	centroidGuessFile.close();

  return (0);
}
