/**
 * constants.h
 * 26.02.2012
 * ccs, cec24@phy.duke.edu
 * 
 * define the sides of the cubes, this is used in loads of places! 
 * if the geometry of the system changes, change it here and rebuild
 * 
 * pcl units are in m, we follow the convention
 */

/** 
 * the horizontal side length (m)*/
const float cubeShortSide = 0.02; 
/**
 * the vertical side length (m) */
const float cubeLongSide = 0.06; 

/**
 * how many points on a long side (estimate) 
 * for making test cubes, either for icp registration or for 
 * testing fit_planes.cpp */
const int nLongSideHighRes = 54; // high res
const int nLongSideLowRes = 27; // high res

const int nShortSideHighRes = 18; // high res
const int nShortSideLowRes = 9; // high res

const int returnNANCentroid = 5;
const int returnGuessNormal = 6;
const int returnNoPlanes = 7;
