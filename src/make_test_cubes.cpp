
int main (int argc, char** argv){
	
	if(argc < 1) {
		std::cerr << "# makes some test cubes for fit_planes.cpp" << std::endl;x
		std::cerr << "# run with <inputcube..path> <outpath> <id>" << std::endl;
	}

	std::string outpath = std::string(argv[1]); 

	pcl::PointCloud<pcl::PointXYZ>::Ptr cubeCloud;

	std::cerr << "# generating cubeCloud" << std::endl;
	// low res
	cubeCloud = genTestCube(9,27); // gen our comparison cube
	
	// apply a series of rotations and output the cube
	// then create a cube with only sides no tops
	// then only tops no sides
	// then random combinations
	// etc..
	

}

/**
 * generate a cuboid centered at the origin
 * 
 * the x,z planes have npointsShort per side
 * the y,z and y,z planes have npointsLong per side
 * the side lengths are set at cubeShortSide and cubeLongSide
 * repsectively
 * 
 * by eye there are ~ 18 scan pts on a short side,
 * so we should have 54 on a long side
 *
 * sideArray turns on the various sides {1,1,1,1,1,1} means we have all of them
 * numbered as in the fn below (top, bottom, 1, 2, 3, 4)
 * and so on
 */

pcl::PointCloud<pcl::PointXYZ>::Ptr genTestCube(int npointsShortSide, int npointsLongSide, int sideArray[6]){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cubeCloud (new pcl::PointCloud<pcl::PointXYZ>);
	const float cubeShortSide = 0.02;
  const float cubeLongSide = 0.06; // seems to be 0.06 not 0.04?

	//const float cubeShortSide = 0.2;
	//const float cubeLongSide = 0.6; // seems to be 0.06 not 0.04?


	int npointsBottomFace = npointsShortSide* npointsShortSide;
	int npointsSideFace = npointsShortSide* npointsLongSide;
	int npointsTotal = 2*npointsBottomFace + 4 *npointsSideFace;
	
	float dxShortSide = cubeShortSide / (float)npointsShortSide;
	float dxLongSide =  cubeLongSide /  (float)npointsLongSide;
	
	std::cerr << "# dxShortSide: " << dxShortSide << std::endl;
	std::cerr << "# dxLongSide: " << dxLongSide << std::endl;
	std::cerr << "# npointsTotal: " << npointsTotal << std::endl;
	
	cubeCloud->width = npointsTotal;
	cubeCloud->height = 1;
	cubeCloud->points.resize(npointsTotal); // allocate space for all the points we need
	
	// make the top and bottom cap faces
	// these go at y = +- cubeLongSide /2 
	// from x,z = -ls/2, -ls/2  to x,z = ls/2, ls/2
	int counter = 0;
	float xval, yval, zval;
	float xOffset, yOffset, zOffset;

	if(sideArray[0]){
	// top face
		yval = cubeLongSide / 2;
		xOffset = - cubeShortSide / 2;
		zOffset = - cubeShortSide / 2;
		for(int i = 0; i < npointsShortSide; i++){
			for(int j = 0; j < npointsShortSide; j++){
				cubeCloud->points[counter].x = i*dxShortSide +  xOffset;
				cubeCloud->points[counter].y = yval;
				cubeCloud->points[counter].z = j*dxShortSide + zOffset;
				counter++;
			}
		}
	}

	if(sideArray[1]){
		// bottom face
		yval = -cubeLongSide / 2;
		xOffset = - cubeShortSide / 2;
		zOffset = - cubeShortSide / 2;
		for(int i = 0; i < npointsShortSide; i++){
			for(int j = 0; j < npointsShortSide; j++){
				cubeCloud->points[counter].x = i*dxShortSide + xOffset;
				cubeCloud->points[counter].y = yval;
				cubeCloud->points[counter].z = j*dxShortSide + zOffset;
				counter++;
			}
		}
	}


	// make each side plane
	// 1) z= -cubeShortSide/2, x: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2
	// 2) z= cubeShortSide/2, x: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2
	// 3) x = -cubeShortSide/2, z: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2
	// 4) x = -cubeShortSide/2, z: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2

	if(sideArray[2]){
		zval = -cubeShortSide / 2;
		xOffset = - cubeShortSide / 2;
		yOffset = - cubeLongSide / 2;
		for(int i = 0; i < npointsShortSide; i++){
			for(int j = 0; j < npointsLongSide; j++){
				cubeCloud->points[counter].x = i*dxShortSide + xOffset;
				cubeCloud->points[counter].y = j*dxLongSide + yOffset;
				cubeCloud->points[counter].z = zval;
				counter++;
			}
		}
	}

	if(sideArray[3]){
		zval = cubeShortSide / 2;
		xOffset = - cubeShortSide / 2;
		yOffset = - cubeLongSide / 2;
		for(int i = 0; i < npointsShortSide; i++){
			for(int j = 0; j < npointsLongSide; j++){
				cubeCloud->points[counter].x = i*dxShortSide + xOffset;
				cubeCloud->points[counter].y = j*dxLongSide +  yOffset;
				cubeCloud->points[counter].z = zval;
				counter++;
			}
		}
	}

	if(sideArray[4]){
		xval = -cubeShortSide / 2;
		zOffset = - cubeShortSide / 2;
		yOffset = - cubeLongSide / 2;
		for(int i = 0; i < npointsShortSide; i++){
			for(int j = 0; j < npointsLongSide; j++){
				cubeCloud->points[counter].x = xval;
				cubeCloud->points[counter].y = j*dxLongSide + yOffset;
				cubeCloud->points[counter].z = i*dxShortSide + zOffset;
				counter++;
			}
		}
	}

	if(sideArary[5]){
		xval = cubeShortSide / 2;
		zOffset = - cubeShortSide / 2;
		yOffset = - cubeLongSide / 2;
		for(int i = 0; i < npointsShortSide; i++){
			for(int j = 0; j < npointsLongSide; j++){
				cubeCloud->points[counter].x = xval;
				cubeCloud->points[counter].y = j*dxLongSide + yOffset;
				cubeCloud->points[counter].z = i*dxShortSide + zOffset;
				counter++;
			}
		}
	}
	

	return cubeCloud;
}
