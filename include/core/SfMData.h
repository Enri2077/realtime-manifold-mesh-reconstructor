/*
 * SfMParser.h
 *
 *  Created on: 16 mar 2016
 *      Author: andrea
 */

#ifndef CAM_PARSERS_SFMPARSER_H_
#define CAM_PARSERS_SFMPARSER_H_

#include <types_reconstructor.hpp>
#include <glm.hpp>

struct SfMData {

  int numPoints_;
  int numCameras_;

  std::vector<glm::vec3> points_;
  std::vector<CameraType> camerasList_;
  std::vector<std::string> camerasPaths_;

  std::vector<std::vector<int> > camViewingPointN_;
  std::vector<std::vector<int> > pointsVisibleFromCamN_;
  std::vector<std::vector<glm::vec2> > point2DoncamViewingPoint_;

  int imageWidth_, imageHeight_;
};

#endif /* CAM_PARSERS_SFMPARSER_H_ */
