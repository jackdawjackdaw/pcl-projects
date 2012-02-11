#include "binfile.h"

/** 
 * dump a cloud into a binfile
 */
int writeBinfileCCS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string filename)
{
	FILE *fptr = fopen(filename.c_str(), "w");
	int i;
	int size = cloud->width  * cloud->height;
	int width  = cloud->width;
	int height = cloud->height;
	float px, py, pz;
	
	fwrite(&size, sizeof(int), 1, fptr);
	fwrite(&width, sizeof(int), 1, fptr);
	fwrite(&height, sizeof(int), 1, fptr);
	for(i = 0; i < size; i++){
		px = cloud->points[i].x;
		py = cloud->points[i].y;
		pz = cloud->points[i].z;
		fwrite(&px, sizeof(float), 1, fptr);
		fwrite(&py, sizeof(float), 1, fptr);
		fwrite(&pz, sizeof(float), 1, fptr);
	}
	fclose(fptr);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr readBinfileCCS(std::string filename)
{
	FILE *fptr = fopen(filename.c_str(), "r");
	int i;
	int size, width, height;
	float px, py, pz;
	float *pointsX, *pointsY, *pointsZ;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	

	fread(&size, sizeof(int), 1, fptr);
	fread(&width, sizeof(int), 1, fptr);
	fread(&height, sizeof(int), 1, fptr);
	
	cloud->height = size;
	cloud->width = width;
	cloud->height = height;
	
	cloud->points.resize(size); // make sure we have enough space
	

	for(i = 0; i < size; ++i){
		fread(&px, sizeof(float), 1, fptr);
		fread(&py, sizeof(float), 1, fptr);
		fread(&pz, sizeof(float), 1, fptr);
		cloud->points[i].x = px;
		cloud->points[i].y = py;
		cloud->points[i].z = pz;
	}
														 

	return(cloud);
}
