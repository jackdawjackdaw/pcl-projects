#include "binfile.h"


int main (int argc, char** argv)
{
	
	if(argc < 2){
		std::cerr << "# run with: <binfile> <pcdfile>\n" << std::endl;
		return EXIT_FAILURE;
	}

	std::string filename = std::string(argv[1]);
	std::string outpath = std::string(argv[2]);
	
	std::cerr << "# reading from: " << filename << std::endl;
	std::cerr << "# output to: " << outpath << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readBinfileCCS(filename);

	std::cerr << "# read width: " << cloud->width << std::endl;
	std::cerr << "# read height: " << cloud->height << std::endl;
	
	pcl::io::savePCDFileASCII(outpath, *cloud);
	
	return EXIT_SUCCESS;
}
