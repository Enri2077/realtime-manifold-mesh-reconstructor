/*
 * OpenMvgParser.cpp
 *
 *  Created on: 16 mar 2016
 *      Author: andrea
 */

#include <OpenMvgParser.h>
#include <Exceptions.hpp>
#include <stdexcept>
#include <rapidjson/reader.h>
#include <utilities.hpp>

OpenMvgParser::OpenMvgParser(std::string path) {
  fileStream_.open(path.c_str());
}

OpenMvgParser::~OpenMvgParser() {
  sfm_data_.points_.clear();
  sfm_data_.camerasList_.clear();
  sfm_data_.camerasPaths_.clear();
  sfm_data_.camViewingPointN_.clear();
  sfm_data_.pointsVisibleFromCamN_.clear();
  sfm_data_.point2DoncamViewingPoint_.clear();
}

void OpenMvgParser::parse() {
  std::string str((std::istreambuf_iterator<char>(fileStream_)), std::istreambuf_iterator<char>());
  document_.Parse(str.c_str());

  try {
    if (!document_.IsObject())
      throw JsonParseException("JsonParseException--> the json file " + fileName_ + " is not valid");
  } catch (JsonParseException& e) {
    std::cerr << e.what() << std::endl;
    std::cout << e.what() << std::endl;
  }

  std::map<int, glm::mat3> intrinsics;
  std::map<int, CameraType> extrinsics;

  parseIntrinsics(intrinsics);
  parseExtrinsics(extrinsics);

  parseViews(intrinsics, extrinsics);

  parsePoints();

}

void OpenMvgParser::parseViews(const std::map<int, glm::mat3> & intrinsics, const std::map<int, CameraType> & extrinsics) {

  std::string basePath(document_["root_path"].GetString());

  try {
    if (!document_.HasMember("views"))
      throw JsonAccessException("JsonAccessException--> error while querying HasMember(views)");
    const rapidjson::Value& camerasJson = document_["views"];
    sfm_data_.numCameras_ = camerasJson.Size();
    sfm_data_.camerasList_.assign(sfm_data_.numCameras_, CameraType());
    sfm_data_.camerasPaths_.assign(sfm_data_.numCameras_, std::string());
    for (rapidjson::SizeType curCam = 0; curCam < camerasJson.Size(); curCam++) {

      std::string local(camerasJson[curCam]["value"]["ptr_wrapper"]["data"]["local_path"].GetString());
      std::string filename(camerasJson[curCam]["value"]["ptr_wrapper"]["data"]["filename"].GetString());

      sfm_data_.camerasList_[curCam].pathImage = basePath + local + filename;
      sfm_data_.camerasPaths_[curCam] = basePath + local + filename;

      sfm_data_.camerasList_[curCam].imageWidth = camerasJson[curCam]["value"]["ptr_wrapper"]["data"]["width"].GetInt();
      sfm_data_.camerasList_[curCam].imageHeight = camerasJson[curCam]["value"]["ptr_wrapper"]["data"]["height"].GetInt();
      int idIntrinsics = camerasJson[curCam]["value"]["ptr_wrapper"]["data"]["id_intrinsic"].GetInt();
      int idExtrinsics = camerasJson[curCam]["value"]["ptr_wrapper"]["data"]["id_pose"].GetInt();

      try {
        glm::mat3 intr = intrinsics.at(idIntrinsics);
        sfm_data_.camerasList_[curCam].intrinsics = intr;
      } catch (std::out_of_range e) {
        std::cerr << "std::out_of_range exception trying to look for intrinsics matrix " << idIntrinsics << std::endl;
      }
      try {
        CameraType camCurrent = extrinsics.at(idExtrinsics);
        glm::mat4 eMatrix(0.0), kMatrix(0.0);
        for (int curR = 0; curR < 3; curR++) {
          for (int curC = 0; curC < 3; curC++) {
            eMatrix[curR][curC] = camCurrent.rotation[curR][curC];
          }
        }
        eMatrix[0][3] = camCurrent.translation[0];
        eMatrix[1][3] = camCurrent.translation[1];
        eMatrix[2][3] = camCurrent.translation[2];
        eMatrix[3][3] = 1.0;

        for (int curR = 0; curR < 3; curR++) {
          for (int curC = 0; curC < 3; curC++) {
            kMatrix[curR][curC] = sfm_data_.camerasList_[curCam].intrinsics[curR][curC];
          }
        }


        sfm_data_.camerasList_[curCam].cameraMatrix =  eMatrix*kMatrix;
        sfm_data_.camerasList_[curCam].rotation = glm::mat3(camCurrent.rotation);
        sfm_data_.camerasList_[curCam].translation = glm::vec3(camCurrent.translation);
        sfm_data_.camerasList_[curCam].center = glm::vec3(camCurrent.center);
       /* std::cout<<"curCam: "<<curCam<<std::endl;
        std::cout<<"Intrinsics A"<<std::endl;
        utilities::printMatrix(kMatrix);
        std::cout<<"Intrinsics B"<<std::endl;
        utilities::printMatrix(sfm_data_.camerasList_[curCam].intrinsics);
        std::cout<<"Rotation"<<std::endl;
        utilities::printMatrix(sfm_data_.camerasList_[curCam].rotation);
        std::cout<<"traslation"<<std::endl;
        utilities::printMatrix(sfm_data_.camerasList_[curCam].translation);
        std::cout<<"Extrinsics"<<std::endl;
        utilities::printMatrix(eMatrix);
        std::cout<<"camera matrix"<<std::endl;
        utilities::printMatrix(sfm_data_.camerasList_[curCam].cameraMatrix);*/

      } catch (std::out_of_range e) {
        std::cerr << "std::out_of_range exception trying to look for extrinsics matrix " << idExtrinsics << std::endl;
      }
    }
  } catch (JsonAccessException& e) {
    std::cerr << e.what() << std::endl;
    std::cout << e.what() << std::endl;

  }

}

void OpenMvgParser::parsePoints() {
  try {

    if (!document_.HasMember("structure"))
      throw JsonAccessException("JsonAccessException--> error while querying HasMember(structure)");
    const rapidjson::Value& structure = document_["structure"];
    sfm_data_.numPoints_ = structure.Size();

    //**********PARSING POINTS
    sfm_data_.camViewingPointN_.assign(sfm_data_.numPoints_, std::vector<int>());
    sfm_data_.point2DoncamViewingPoint_.assign(sfm_data_.numPoints_, std::vector<glm::vec2>());
    sfm_data_.pointsVisibleFromCamN_.assign(sfm_data_.numCameras_, std::vector<int>());
    sfm_data_.points_.assign(sfm_data_.numPoints_, glm::vec3());
    if (!structure.IsArray())
      throw JsonAccessException("JsonAccessException--> error while querying structure.IsArray()");

    for (rapidjson::SizeType curPoint = 0; curPoint < structure.Size(); curPoint++) { // Uses SizeType instead of size_t

      if (!structure[curPoint].IsObject())
        throw JsonAccessException("JsonAccessException--> error while querying structure[i].IsObject()");
      //      if (!structure[i].HasMember("key"))
      //        throw JsonAccessException("JsonAccessException--> error while querying structure[i].HasMember(key)");
      //std::cout << "structure[i][key]. " << structure[i]["key"].GetInt() << std::endl;

      if (!structure[curPoint].HasMember("value"))
        throw JsonAccessException("JsonAccessException--> error while querying structure[i].HasMember(value)");
      if (!structure[curPoint]["value"].HasMember("X"))
        throw JsonAccessException("JsonAccessException--> error while querying structure[i][value].HasMember(X)");

      const rapidjson::Value& X = structure[curPoint]["value"]["X"];
      if (!X.IsArray())
        throw JsonAccessException("JsonAccessException--> error while querying X.IsArray()");

      if (!X[0].IsDouble())
        throw JsonAccessException("JsonAccessException--> error while querying X0.IsDouble()");

      if (!X[1].IsDouble())
        throw JsonAccessException("JsonAccessException--> error while querying X1.IsDouble()");

      if (!X[2].IsDouble())
        throw JsonAccessException("JsonAccessException--> error while querying X2.IsDouble()");

      float x0 = X[0].GetFloat();
      float x1 = X[1].GetFloat();
      float x2 = X[2].GetFloat();

      sfm_data_.points_[curPoint] = glm::vec3(x0, x1, x2);

      if (!structure[curPoint]["value"].HasMember("observations"))
        throw JsonAccessException("JsonAccessException--> error while querying structure[i][value].HasMember(observations)");

      const rapidjson::Value& observations = structure[curPoint]["value"]["observations"];

      if (!observations.IsArray())
        throw JsonAccessException("JsonAccessException--> error while querying observations.IsArray()");

      for (rapidjson::SizeType curId = 0; curId < observations.Size(); curId++) {
        if (!observations[curId].HasMember("key"))
          throw JsonAccessException("JsonAccessException--> error while querying observations[i].HasMember(key)");
        //std::cout << "observations[" << curId << "].HasMember(key)= " << observations[curId]["key"].GetInt() << std::endl;

        int curCam = observations[curId]["key"].GetInt();

        sfm_data_.camViewingPointN_[curPoint].push_back(curCam);
        sfm_data_.pointsVisibleFromCamN_[curCam].push_back(curPoint);

        if (!observations[curId].HasMember("value"))
          throw JsonAccessException("JsonAccessException--> error while querying observations[curId].HasMember(value)");

        if (!observations[curId]["value"].HasMember("x"))
          throw JsonAccessException("JsonAccessException--> error while querying observations[curId][value].HasMember(x)");

        const rapidjson::Value& pt2D = observations[curId]["value"]["x"];
        sfm_data_.point2DoncamViewingPoint_[curPoint].push_back(glm::vec2(pt2D[0].GetDouble(), pt2D[1].GetDouble()));
      }
    }
  } catch (JsonAccessException& e) {
    std::cerr << e.what() << std::endl;
    std::cout << e.what() << std::endl;

  } catch (JsonParseException& e) {
    std::cerr << e.what() << std::endl;
    std::cout << e.what() << std::endl;
  }
}

void OpenMvgParser::parseIntrinsics(std::map<int, glm::mat3> & intrinsics) {

  try {

    if (!document_.HasMember("intrinsics"))
      throw JsonAccessException("JsonAccessException--> error while querying HasMember(views)");
    const rapidjson::Value& intrinsicsJson = document_["intrinsics"];

    for (rapidjson::SizeType curInt = 0; curInt < intrinsicsJson.Size(); curInt++) {
      int key = intrinsicsJson[curInt]["key"].GetInt();
      glm::mat3 temp(0.0);
      temp[0][0] = intrinsicsJson[curInt]["value"]["ptr_wrapper"]["data"]["focal_length"].GetFloat();
      temp[1][1] = intrinsicsJson[curInt]["value"]["ptr_wrapper"]["data"]["focal_length"].GetFloat();
      temp[0][2] = intrinsicsJson[curInt]["value"]["ptr_wrapper"]["data"]["principal_point"][0].GetFloat();
      temp[1][2] = intrinsicsJson[curInt]["value"]["ptr_wrapper"]["data"]["principal_point"][1].GetFloat();
      temp[2][2] = 1.0;

      intrinsics.insert(std::pair<int, glm::mat3>(key, temp));
    }

  } catch (JsonAccessException& e) {
    std::cerr << e.what() << std::endl;
    std::cout << e.what() << std::endl;

  }
}

void OpenMvgParser::parseExtrinsics(std::map<int, CameraType> & extrinsics) {

  try {

    if (!document_.HasMember("extrinsics"))
      throw JsonAccessException("JsonAccessException--> error while querying HasMember(extrinsics)");
    const rapidjson::Value& extrinsicsJson = document_["extrinsics"];

    for (rapidjson::SizeType curInt = 0; curInt < extrinsicsJson.Size(); curInt++) {
      int key = extrinsicsJson[curInt]["key"].GetInt();
      CameraType temp;

      for (int curR = 0; curR < 3; curR++) {
        for (int curC = 0; curC < 3; curC++) {
          temp.rotation[curR][curC] = extrinsicsJson[curInt]["value"]["rotation"][curR][curC].GetFloat();
        }
      }

      for (int curR = 0; curR < 3; curR++) {
        temp.center[curR] = extrinsicsJson[curInt]["value"]["center"][curR].GetFloat();
      }
      temp.translation = -temp.center * temp.rotation;

      extrinsics.insert(std::pair<int, CameraType>(key, temp));
    }

  } catch (JsonAccessException& e) {
    std::cerr << e.what() << std::endl;
    std::cout << e.what() << std::endl;

  }
}
