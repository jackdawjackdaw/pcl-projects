#include <pcl/point_types.h>

// holds info about a plane,
// particularly: the centre, the x,y,z ranges, if its a vertical (long side) or horizontal plane
// estimated normals
struct planeInfo{
	/** 
	 * the estimated centre of the plane */
	pcl::PointXYZ center;
	/**
	 * the estimated normal of the plane 
	 * the sign is initially not known */
	pcl::PointXYZ normal; 

	
	/**
	 * the ranges for the x y z extents */
	double xrange[2];
	double yrange[2];
	double zrange[2]; 
	/**
   * is this a long edge plane in the xy  
	 * dims would be 6 x 2 */
	bool vertical; 
	/**
	 * is this an end cap, smaller 
	 * dims are: 2 x 2 */
	bool horizontal;
	/**
	 * coeffs of the ransac model that produced this */
	double coeffs[4];
	/**
	 * the distance of this surface from the centroid (supossedly) */
	double radius;

	/**
	 * as far as we can deduce, should the normal be positive or negatively signed
	 */
	float normalSign;
	
	/**
	 * the 2d convex hull defining this face*/
	pcl::PointCloud<pcl::PointXYZ>  cloud_hull;
	
};

