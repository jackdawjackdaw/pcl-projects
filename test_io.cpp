#include "stdio.h"
#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/**
 * try and read a pcd file without fucking up
 */
int main (int argc, char** argv){
	
	pcl::PointCloud<pcl::PointXYZ> cloud;

	std::string filename = std::string(argv[1]);
	// ok this works
	std::cerr << "loading file: " << filename << std::endl;
	pcl::io::loadPCDFile(filename, cloud);
	
	std::cerr << "height: " << cloud.height << " width: " << cloud.width << std::endl;

	std::cerr << "done" << std::endl;
	
	return EXIT_SUCCESS;
}
