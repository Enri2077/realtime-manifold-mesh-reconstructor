/*
 * ORBParser.h
 *
 *  Created on: 26 Mar 2016
 *      Author: enrico
 */

#ifndef CAM_PARSERS_ORBPARSER_H_
#define CAM_PARSERS_ORBPARSER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <rapidjson/document.h>
#include <CameraPointsCollection.h>
#include <types_reconstructor.hpp>


class ORBParser {
public:
  ORBParser(std::string path);
  virtual ~ORBParser();
  virtual void parse();
  std::string getStats();
  std::string getDataCSV();
  std::string getDataSPlot();

  const CameraPointsCollection& getData() const {
    return ORB_data_;
  }

private:
  void parseViews(const std::map<int,glm::mat3> & intrinsics, const std::map<int,CameraType> & extrinsics);
  void parseIntrinsics(std::map<int,glm::mat3> & intrinsics);
  void parseExtrinsics(std::map<int,CameraType> & extrinsics);
  void parsePoints();


  rapidjson::Document document_;
  std::string fileName_;
  std::ifstream fileStream_;
  CameraPointsCollection ORB_data_;

};

#endif /* CAM_PARSERS_ORBPARSER_H_ */
