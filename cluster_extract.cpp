#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


int 
main (int argc, char** argv)
{
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
	free(pointsX);
	free(pointsY);
	free(pointsZ);


  // Read in the cloud data
  //pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //reader.read ("table_scene_lms400.pcd", *cloud);
	//reader.read ("test_pcd.pcd", *cloud);
  // Fill in the cloud data
	// we know this from the pfm file

	cloud->width    = nPoints;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = pointsX[i];
    cloud->points[i].y = pointsY[i];
    cloud->points[i].z = pointsZ[i];
  }

	
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

	// ccs, do we need to filter the data, not sure what the downsampling is doing yet

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.001f, 0.001f, 0.001f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  // pcl::SACSegmentation<pcl::PointXYZ> seg;
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (100);
  // seg.setDistanceThreshold (0.02);

  // int i=0, nr_points = (int) cloud_filtered->points.size ();
  // while (cloud_filtered->points.size () > 0.3 * nr_points)
  // {
  //   // Segment the largest planar component from the remaining cloud
  //   seg.setInputCloud(cloud_filtered);
  //   seg.segment (*inliers, *coefficients); //*
  //   if (inliers->indices.size () == 0)
  //   {
  //     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  //     break;
  //   }

  //   // Extract the planar inliers from the input cloud
  //   pcl::ExtractIndices<pcl::PointXYZ> extract;
  //   extract.setInputCloud (cloud_filtered);
  //   extract.setIndices (inliers);
  //   extract.setNegative (false);

  //   // Write the planar inliers to disk
  //   extract.filter (*cloud_plane); //*
  //   std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

  //   // Remove the planar inliers, extract the rest
  //   extract.setNegative (true);
  //   extract.filter (*cloud_filtered); //*
  // }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.01); // 1cmm
  ec.setMinClusterSize (50); // 10 is probably too low
  ec.setMaxClusterSize (5000);
  ec.setSearchMethod (tree);
  ec.setInputCloud( cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);
}
