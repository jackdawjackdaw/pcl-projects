#include "binfile.h"


int main (int argc, char** argv)
{
	
	if(argc < 2){
		std::cerr << "# run with: <pcdfile> <binfile>\n" << std::endl;
		return EXIT_FAILURE;
	}

	std::string filename = std::string(argv[1]);
	std::string outpath = std::string(argv[2]);
	
	std::cerr << "# reading from: " << filename << std::endl;
	std::cerr << "# output to: " << outpath << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile(filename, *cloud);

	std::cerr << "# read width: " << cloud->width << std::endl;
	std::cerr << "# read height: " << cloud->height << std::endl;
	
	//pcl::io::savePCDFileASCII(outpath, *cloud);
	writeBinfileCCS(cloud, outpath);

	return EXIT_SUCCESS;
}
