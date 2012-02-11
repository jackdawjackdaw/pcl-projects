#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#include "stdio.h"
#include <cstring>
#include <string>
#include <iostream>
#include <sstream>


int writeBinfileCCS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string filename);

pcl::PointCloud<pcl::PointXYZ>::Ptr readBinfileCCS(std::string filename);
