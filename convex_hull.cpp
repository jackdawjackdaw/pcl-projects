#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;

	reader.read(argv[1], *cloud); 

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud);
	//chull.setAlpha(0.1);
	chull.setComputeAreaVolume(true);
	chull.reconstruct(*cloud_hull);

	std::cout << "hull vol: " << chull.getTotalVolume() << "hull npts: " << cloud_hull->points.size() << std::endl;
	
	pcl::PCDWriter writer;
  writer.write ("output_hull.pcd", *cloud_hull, false);

	return(0);
}
