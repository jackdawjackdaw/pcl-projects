#include "stdio.h"

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>



#include "binfile.h"

// for safe path stuff
#include <boost/filesystem.hpp>>

std::vector<boost::filesystem::path>* findFilesInPath(std::string pathString, std::string extString);


/**
 * ccs,
 *
 * read all the pcd files in  
 * directory, merge them together using icp output to 
 * the given combined name
 * 
 * no idea if this works....
 */
int main (int argc, char** argv){
	if(argc < 2){
		std::cerr << "# reads all the *.cbin files in arg1, attempts to register them" << std::endl;
		std::cerr << "# outputs the merged file as a cbin to arg2" << std::endl;
		std::cerr << "# args: infolder, outname" << std::endl;
		return EXIT_FAILURE;
	}
	

	std::string outName = argv[2];

	boost::filesystem::path p;

	std::vector<boost::filesystem::path>* inputFilePaths = findFilesInPath(std::string(argv[1]), ".cbin");

	for(std::vector<boost::filesystem::path>::iterator it = inputFilePaths->begin(); it != inputFilePaths->end(); ++it){
		std::cout << "# processing: " << *it  << std::endl;
	}

	int nRegClouds = inputFilePaths->size();

	// init a vec of ptrs to point clouds
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudVec;

	pcl::PointCloud<pcl::PointXYZ>::Ptr p1, p2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalPtr (new pcl::PointCloud<pcl::PointXYZ>);

	cloudVec.reserve(nRegClouds);

	std::vector<boost::filesystem::path>::iterator it;
	it = inputFilePaths->begin();
	std::string *instring;
	for(int index = 0; index < nRegClouds; ++index){
		instring = new std::string((*it).native());
		std::cout << "# loading: " << *instring  << std::endl;
		//pcl::io::loadPCDFile(*instring, (*jt)); (this is still being painful)
		cloudPtr = readBinfileCCS(*instring);
		std::cerr << "# read width: " << cloudPtr->width << std::endl;
	  std::cerr << "# read height: " << cloudPtr->height << std::endl;
		cloudVec.push_back(cloudPtr);
		cloudPtr.reset();
		++it;
		delete instring;
	}
	

	std::cerr << "# starting icp" << std::endl;
	int i = 0;

	p1.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloudVec[0]));
	for(int i = 0; i < nRegClouds-1; i++){
		p2.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloudVec[i+1]));

	
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputCloud(p1);
		icp.setInputTarget(p2);
		icp.setTransformationEpsilon(1e-4);
		icp.setMaximumIterations(12);
		icp.align(*finalPtr);

		std::cout << "has converged:" << icp.hasConverged() << " score: " <<
			icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;
		p1 = finalPtr;
	}

	writeBinfileCCS(finalPtr, outName);

	return EXIT_SUCCESS;
}


std::vector<boost::filesystem::path>* findFilesInPath(std::string pathString, std::string extString){

	std::vector<boost::filesystem::path> *inputFilePaths = new std::vector<boost::filesystem::path>;	boost::filesystem::path p;
	boost::filesystem::path dataPath(pathString);
	
	try{
		if(boost::filesystem::exists(dataPath)){
			if(boost::filesystem::is_directory(dataPath)){
				for(boost::filesystem::directory_iterator it = boost::filesystem::directory_iterator(dataPath);	it != boost::filesystem::directory_iterator(); ++it){
					//std::cout << *it  << std::endl;
					p = (*it).path();
					//std::cout << p.extension() << std::endl;
					if(p.extension() == extString){
						//std::cout << "# caught a file " << std::endl;
						inputFilePaths->push_back(p);
					}
				}

			}
		} else {
			std::cerr << dataPath << "does not exist or is not a dir\n";
			exit(EXIT_FAILURE);
		}
	}
  catch (const boost::filesystem::filesystem_error& ex)
  {
		std::cerr << ex.what() << '\n';
		exit(EXIT_FAILURE);
  }	

	if(inputFilePaths->size() == 0){
		std::cerr << "found no suitable files in: " << dataPath << std::endl;
		exit(EXIT_FAILURE);
	}

	
	return inputFilePaths;
}
