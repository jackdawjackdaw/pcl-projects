#define _GLIBCXX_FULLY_DYNAMIC_STRING 1

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/**
 * try and read a pcd file without fucking up
 */
int main (int argc, char** argv){
	
	pcl::PointCloud<pcl::PointXYZ> cloud;

	cloud.height = 10;
	cloud.width = 1;
	cloud.points.resize(10);

	cloud.points[1].x = 1.0;
	cloud.points[1].y = 1.0;
	cloud.points[1].z = 1.0;

	std::string filename = std::string(argv[1]);
	// ok this works if you just load, but not if you do two
	// io actions...
	std::cerr << "loading file: " << filename << std::endl;
	pcl::io::loadPCDFile(filename, cloud);
	//pcl::PCDReader *reader = new pcl::PCDReader;
	//reader->read(filename , cloud);
	//delete reader;
	
	std::cerr << "height: " << cloud.height << " width: " << cloud.width << std::endl;

	std::cerr << "done" << std::endl;


	pcl::io::savePLYFile("test.ply", cloud);	
	return EXIT_SUCCESS;
}

// void writeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string filename){
// 	pcl::PCDWriter *writer = new pcl::PCDWriter;
// 	writer->write("test.pcd", *cloud);
// 	delete writer;
// }
