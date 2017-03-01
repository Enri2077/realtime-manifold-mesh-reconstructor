#ifndef UTILITIES_HPP_
#define UTILITIES_HPP_

#include <opencv2/core/core.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <glm.hpp>
#include <string>
#include <types_reconstructor.hpp>
//#include <SfMData.h>

namespace utilities {
/*compute the Euclidean distance among differen tpes of points*/
float distanceEucl(cv::Point3f p1, cv::Point3f p2);
double distanceEucl(cv::Point3d p1, cv::Point3d p2);
double distanceEucl(PointD3 p1, PointD3 p2);
double distanceEuclSquared(PointD3 p1, PointD3 p2);
double distanceEucl(double x1, double x2, double y1, double y2, double z1, double z2);
float distanceEucl(float x1, float x2, float y1, float y2, float z1, float z2);

//compute the string representing the frame number with a specified padding
std::string getFrameNumber(int curFrame, int digitIdxLength);

//write an OB files with only vertices
void writeObj(std::vector<glm::vec3> &vertices, std::string path);

/*
 *  Output the vertices of a tetrahedron
 *   param cell specifies the tetrahedron
 *   param printVertex if not specified or -1 the function will print all the vertices,
 *         otherwise it specifies the index of the vertices that will be written
 *   return the string containing informations about vertices
 */
std::string printTet(Delaunay3::Cell_handle &cell, int printVertex = -1);

/**Outputs the string containing the tetrahedron in the OFF-file format
 **/
std::string printTetOFF(Delaunay3::Cell_handle &cell);
/**
 * Print on standard output the matrix specified
 */
void printMatrix(glm::mat4 matrix);
void printMatrix(std::string message, glm::mat4 matrix);
void printMatrix(glm::mat3 matrix);
void printMatrix(std::string message, glm::mat3 matrix);
void printMatrix(glm::vec3 vector);
void printMatrix(std::string message, glm::vec3 vector);
void printMatrix(glm::vec4 vector);
void printMatrix(std::string message, glm::vec4 vector);

/*Stores the visibility rays in a ply file named initVisT.ply
 *
 *
 * param camCenters list of camera centers in world coordinate
 * param points list of points centers in camera coordinates
 * pointsVisibleFromCamN visibility information: vector of i int vectors; each i-th int vectors is the list
 *                       of points indices  visible from th ei-th cam
 */
void saveVisibilityPly(const std::vector<glm::vec3> &camCenters, const std::vector<glm::vec3> & points,
    const std::vector<std::vector<int> >& visibility,const std::string &name = "visibility", bool pointsVisibleFromCamN = true);

//void saveVisibilityPly(const SfMData &sfm_data,const std::string &name = "visibility");

/*computes rotation around X axis*/
glm::mat3 rotX(float alpha);

/*computes rotation around Y axis*/
glm::mat3 rotY(float alpha);

/*computes rotation around Z axis*/
glm::mat3 rotZ(float alpha);

/**
 * Read one line in the configuration file and stores the parameter in the value variable
 */
void readLineAndStore(std::ifstream &configFile, bool &value);
void readLineAndStore(std::ifstream &configFile, int &value);
void readLineAndStore(std::ifstream &configFile, double &value);
void readLineAndStore(std::ifstream &configFile, float &value);
void readLineAndStore(std::ifstream &configFile, std::string &value);

/**
 * check if a point is in a given position, with a bit of floating (un-)precision taken into account
 */
bool checkPointInPosition(PointD3 &p, glm::vec3 &position);
bool checkPointInPosition(PointD3 &p, float x, float y, float z);

}  // namespace utils

#endif /* UTILITIES_HPP_ */
