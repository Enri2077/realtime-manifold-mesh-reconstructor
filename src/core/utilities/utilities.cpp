#include <utilities.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

namespace utilities {

double distanceEucl(cv::Point3d p1, cv::Point3d p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

float distanceEucl(cv::Point3f p1, cv::Point3f p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

double distanceEucl(double x1, double x2, double y1, double y2, double z1, double z2) {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}
float distanceEucl(float x1, float x2, float y1, float y2, float z1, float z2) {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}

double distanceEucl(PointD3 p1, PointD3 p2) {
  return sqrt((p1.x() - p2.x()) * (p1.x() - p2.x()) + (p1.y() - p2.y()) * (p1.y() - p2.y()) + (p1.z() - p2.z()) * (p1.z() - p2.z()));
}

double distanceEuclSquared(PointD3 p1, PointD3 p2) {
  return std::pow(p1.x()-p2.x(), 2) + std::pow(p1.y()-p2.y(), 2) + std::pow(p1.z()-p2.z(), 2);
}

std::string getFrameNumber(int curFrame, int digitIdxLength) {
  std::ostringstream curNumber;
  if (digitIdxLength > 0) {
    int n = curFrame;
    int curNumOfDigit = curFrame == 0 ? 1 : 0;
    while (n > 0) {
      n /= 10;
      ++curNumOfDigit;
    }
    while (curNumOfDigit < digitIdxLength) {
      curNumber << "0";
      curNumOfDigit++;
    }
  }
  curNumber << curFrame;
  return curNumber.str();
}

void writeObj(std::vector<glm::vec3> &vertices, std::string path) {
  std::ofstream fileOut(path);
  for (auto v : vertices) {
    fileOut << "v " << v.x << " " << v.y << " " << v.z << " " << std::endl;
  }

  for (int curF = 0; curF < vertices.size(); curF += 3) {
    fileOut << "f " << curF + 1 << " " << curF + 2 << " " << curF + 3 << " " << std::endl;
  }

  fileOut.close();
}

std::string printTet(Delaunay3::Cell_handle &cell, int printVertex) {
  std::stringstream temp;
  std::vector<int> idxVert;
  if (printVertex == -1) {
    idxVert.push_back(0);
    idxVert.push_back(1);
    idxVert.push_back(2);
    idxVert.push_back(3);
  } else {
    idxVert.push_back(printVertex);
  }

  for (auto curV : idxVert) {
    temp << "V" << curV << ": ";
    temp << "(" << cell->vertex(curV)->point().x() << ", ";
    temp << cell->vertex(curV)->point().y() << ", ";
    temp << cell->vertex(curV)->point().z() << ") ";
  }
  return temp.str();
}

std::string printTetOFF(Delaunay3::Cell_handle &cell) {
  std::stringstream temp;
  std::vector<int> idxVert;
  temp << "OFF" << std::endl;
  temp << "4 4 0" << std::endl;

  for (int curV = 0; curV < 4; ++curV) {
    temp << cell->vertex(curV)->point().x() << " ";
    temp << cell->vertex(curV)->point().y() << " ";
    temp << cell->vertex(curV)->point().z() << std::endl;

  }
  temp << "3 0 1 2" << std::endl;
  temp << "3 1 2 3" << std::endl;
  temp << "3 0 1 3" << std::endl;
  temp << "3 0 2 3" << std::endl;

  return temp.str();
}

glm::mat3 rotX(float alpha) {
  glm::mat3 rot; //set identity matrix
  rot[1][1] = cos(alpha);
  rot[1][2] = -sin(alpha);
  rot[2][1] = sin(alpha);
  rot[2][2] = cos(alpha);
  return rot;
}

glm::mat3 rotY(float alpha) {
  glm::mat3 rot; //set identity matrix
  rot[0][0] = cos(alpha);
  rot[0][2] = sin(alpha);
  rot[2][0] = -sin(alpha);
  rot[2][2] = cos(alpha);
  return rot;
}

void printMatrix(glm::mat4 matrix) {
  for (int curRow = 0; curRow < 4; ++curRow) {
    for (int curCol = 0; curCol < 4; ++curCol) {
      std::cout << matrix[curRow][curCol] << " ";
    }
    std::cout << std::endl;
  }
}
void printMatrix(const std::string message, glm::mat4 matrix) {
  std::cout << message << std::endl;
  for (int curRow = 0; curRow < 4; ++curRow) {
    for (int curCol = 0; curCol < 4; ++curCol) {
      std::cout << matrix[curRow][curCol] << " ";
    }
    std::cout << std::endl;
  }
}
void printMatrix(glm::mat3 matrix) {
  for (int curRow = 0; curRow < 3; ++curRow) {
    for (int curCol = 0; curCol < 3; ++curCol) {
      std::cout << matrix[curRow][curCol] << " ";
    }
    std::cout << std::endl;
  }
}

void printMatrix(std::string message, glm::mat3 matrix) {
  std::cout << message << std::endl;
  for (int curRow = 0; curRow < 3; ++curRow) {
    for (int curCol = 0; curCol < 3; ++curCol) {
      std::cout << matrix[curRow][curCol] << " ";
    }
    std::cout << std::endl;
  }
}

void printMatrix(glm::vec3 vector) {
  for (int curIdx = 0; curIdx < 3; ++curIdx) {
    std::cout << vector[curIdx] << " ";
  }
  std::cout << std::endl;
}

void printMatrix(std::string message, glm::vec4 vector) {
  std::cout << message << std::endl;
  printMatrix(vector);
}
void printMatrix(glm::vec4 vector) {
  for (int curIdx = 0; curIdx < 4; ++curIdx) {
    std::cout << vector[curIdx] << " ";
  }
  std::cout << std::endl;
}

void printMatrix(std::string message, glm::vec3 vector) {
  std::cout << message << std::endl;
  for (int curIdx = 0; curIdx < 3; ++curIdx) {
    std::cout << vector[curIdx] << " ";
  }
  std::cout << std::endl;
}

glm::mat3 rotZ(float alpha) {
  glm::mat3 rot; //set identity matrix
  rot[0][0] = cos(alpha);
  rot[0][1] = -sin(alpha);
  rot[1][0] = sin(alpha);
  rot[1][1] = cos(alpha);
  return rot;
}

void readLineAndStore(std::ifstream &configFile, bool &value) {
  std::string line;
  int valueRead;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> valueRead;
  if (valueRead == 0) {
    value = false;
  } else {
    value = true;
  }
}

void readLineAndStore(std::ifstream &configFile, int &value) {
  std::string line;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> value;
}

void readLineAndStore(std::ifstream &configFile, double &value) {
  std::string line;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> value;
}
void readLineAndStore(std::ifstream &configFile, float &value) {
  std::string line;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> value;
}

void readLineAndStore(std::ifstream &configFile, std::string &value) {
  std::string line;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> value;
  std::cout << iss.str() << std::endl;
  if (value.at(0) == '#') {
    value = std::string("");
  }
}

//void saveVisibilityPly(const SfMData& sfm_data,const std::string &name) {
//
//  std::vector<glm::vec3> camCenters;
//  std::vector<glm::vec3> points;
//  for (auto c : sfm_data.camerasList_) {
//    camCenters.push_back(c.center);
//  }
//
//  for (auto p : sfm_data.points_) {
//    points.push_back(p);
//  }
//
//  utilities::saveVisibilityPly(camCenters, points, sfm_data.pointsVisibleFromCamN_, name);
//}

void saveVisibilityPly(const std::vector<glm::vec3> &camCenters, const std::vector<glm::vec3> & points, const std::vector<std::vector<int> >& visibility,
    const std::string &name, bool pointsVisibleFromCamN) {

  std::ofstream visFile(name+".ply");
//
  int numVisRays = 0;
  for (int curIdx = 0; curIdx < visibility.size(); ++curIdx) {
    numVisRays += visibility[curIdx].size();
  }
  std::cout << "Writing initVis_" << std::endl;
  std::cout << "numVisRays_: " << numVisRays << std::endl;
  std::cout << "camCenters_.size(): " << camCenters.size() << std::endl;

  visFile << "ply" << std::endl << "format ascii 1.0" << std::endl;
  visFile << "element vertex " << points.size() + camCenters.size() << std::endl;
  visFile << "property float x " << std::endl << "property float y " << std::endl << "property float z " << std::endl;
  visFile << "element edge " << numVisRays << std::endl;
  visFile << "property int vertex1 " << std::endl << "property int vertex2" << std::endl;
  visFile << "end_header" << std::endl;

  for (auto curCam : camCenters) {
    visFile << curCam.x << " " << curCam.y << " " << curCam.z << std::endl;
  }

  for (auto curPt : points) {
    visFile << curPt.x << " " << curPt.y << " " << curPt.z << std::endl;
  }

  if (pointsVisibleFromCamN) {
    for (int curCamIdx = 0; curCamIdx < camCenters.size(); ++curCamIdx) {

      for (auto curPtIdx : visibility[curCamIdx]) {

        visFile << curCamIdx << " " << camCenters.size() + curPtIdx << std::endl;
      }
    //  std::cout << "Cam: " << curCamIdx << "DONE" << std::endl;

    }
  } else {
    for (int curPtIdx = 0; curPtIdx < points.size(); ++curPtIdx) {

      for (auto curCamIdx : visibility[curPtIdx]) {

        visFile << curCamIdx << " " << camCenters.size() + curPtIdx << std::endl;
      }
      //std::cout << "Point: " << curPtIdx << "DONE" << std::endl;

    }
  }
  visFile.close();
}

bool checkPointInPosition(PointD3 &p, glm::vec3 &position) {

  float xabs = (float) (position.x * 0.001) > 0 ? (position.x * 0.001) : (-position.x * 0.001);
  float yabs = (float) (position.y * 0.001) > 0 ? (position.y * 0.001) : (-position.y * 0.001);
  float zabs = (float) (position.z * 0.001) > 0 ? (position.z * 0.001) : (-position.z * 0.001);

  float pXmin = position.x - xabs;
  float pXmax = position.x + xabs;
  float pYmin = position.y - yabs;
  float pYmax = position.y + yabs;
  float pZmin = position.z - zabs;
  float pZmax = position.z + zabs;

  bool a = pXmin < p.x();
  bool b = p.x() < pXmax;
  bool c = pYmin < p.y();
  bool d = p.y() < pYmax;
  bool e = pZmin < p.z();
  bool f = p.z() < pZmax;

  return (pXmin < p.x() && p.x() < pXmax && pYmin < p.y() && p.y() < pYmax && pZmin < p.z() && p.z() < pZmax);

}

bool checkPointInPosition(PointD3 &p, float x, float y, float z) {
  glm::vec3 position(x, y, z);
  return checkPointInPosition(p, position);
}

}

