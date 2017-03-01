#include <ORBIncrementalParser.h>
#include <Exceptions.hpp>
#include <stdexcept>
#include <rapidjson/reader.h>
#include <utilities.hpp>

#define COMMA <<", "<<
#define SPACE <<" "<<

ORBIncrementalParser::ORBIncrementalParser(std::string path, ManifoldReconstructionConfig conf) {
	fileStream_.open(path.c_str());
	conf_ = conf;

	std::string str((std::istreambuf_iterator<char>(fileStream_)), std::istreambuf_iterator<char>());
	document_.Parse(str.c_str());

	try {
		if (!document_.IsObject()) throw JsonParseException("JsonParseException--> the json file " + fileName_ + " is not valid");
	} catch (JsonParseException& e) {
		std::cerr << e.what() << std::endl;
		std::cout << e.what() << std::endl;
	}

	basePath_ = std::string(document_["root_path"].GetString());
	if (!document_.HasMember("slam_data_version") || (document_["slam_data_version"] != "0.2" && document_["slam_data_version"] != "0.3")) {
		std::cerr << "incorrect data version" << std::endl << document_["slam_data_version"].GetString() << std::endl;
	}

	jsonViewIndex_ = 0;
	jsonViewsArray_ = document_["views"];

	parseIntrinsics();
}

ORBIncrementalParser::~ORBIncrementalParser() {
	ORB_data_.clear();
}

int ORBIncrementalParser::numCameras() {
	return jsonViewsArray_.Size();
}

CameraType* ORBIncrementalParser::nextCamera() {
	if (jsonViewIndex_ < jsonViewsArray_.Size()) {
		const rapidjson::Value& jsonView = jsonViewsArray_[jsonViewIndex_];
		jsonViewIndex_++;

		long unsigned int cameraId = jsonView["viewId"].GetInt();

		CameraType* camera;

		if (ORB_data_.hasCamera(cameraId)) {
			camera = ORB_data_.getCamera(cameraId);
		}else{
			camera = new CameraType();
			camera->idCam = cameraId;
			ORB_data_.addCamera(camera);
		}


		std::string local(jsonView["local_path"].GetString());
		std::string filename(jsonView["filename"].GetString());
		camera->pathImage = basePath_ + local + filename;

		camera->imageWidth = jsonView["width"].GetInt();
		camera->imageHeight = jsonView["height"].GetInt();

		glm::mat3 rotation;
		glm::vec3 center;
		glm::vec3 translation;
		glm::mat4 eMatrix(0.0), kMatrix(0.0);
		glm::mat3 intrinsic = intrinsics_.at(jsonView["id_intrinsic"].GetInt());

		for (int curR = 0; curR < 3; curR++) {
			center[curR] = jsonView["extrinsic"]["center"][curR].GetFloat();
		}

		for (int curR = 0; curR < 3; curR++)
			for (int curC = 0; curC < 3; curC++) {
				kMatrix[curR][curC] = intrinsic[curR][curC];
			}

		for (int curR = 0; curR < 3; curR++)
			for (int curC = 0; curC < 3; curC++) {
				eMatrix[curR][curC] = jsonView["extrinsic"]["rotation"][curR][curC].GetFloat();
				rotation[curR][curC] = jsonView["extrinsic"]["rotation"][curR][curC].GetFloat();
			}

		translation = -center * rotation;

		eMatrix[0][3] = translation[0];
		eMatrix[1][3] = translation[1];
		eMatrix[2][3] = translation[2];
		eMatrix[3][3] = 1.0;

		camera->cameraMatrix = eMatrix * kMatrix;
		camera->rotation = glm::mat3(rotation);
		camera->translation = glm::vec3(translation);
		camera->center = glm::vec3(center);
		camera->intrinsics = intrinsic;
		// TODO check camera properties are correct

		camera->erasedPoints.clear();
		camera->erasedPoints.insert(camera->visiblePointsT.begin(), camera->visiblePointsT.end());

		if(camera->idReconstruction >= 0){
			// Erase the old camera pointers in the points
			for(auto p : camera->visiblePointsT){
				p->viewingCams.erase(camera);
			}

			camera->visiblePointsT.clear();
			//camera->visiblePoints.clear();
		}

		// For each point
		const rapidjson::Value& jsonObservationsArray = jsonView["observations"];
		for (rapidjson::SizeType jsonObservationIndex = 0; jsonObservationIndex < jsonObservationsArray.Size(); jsonObservationIndex++) {
			const rapidjson::Value& jsonObservationObject = jsonObservationsArray[jsonObservationIndex];

			long unsigned int pointId = jsonObservationObject["pointId"].GetInt();
			PointType* point;

			if (ORB_data_.hasPoint(pointId)) {
				point = ORB_data_.getPoint(pointId);

				//std::cout << "UPDATE idPoint: "<<point->idPoint << "\tidReconstruction: "<<point->idReconstruction << "\tgetNunmberObservation: "<<point->getNunmberObservation() << std::endl;

				ORB_data_.addVisibility(camera, point);
				camera->erasedPoints.erase(point);

			} else {
				point = new PointType();
				point->idPoint = pointId;
				ORB_data_.addPoint(point);
				ORB_data_.addVisibility(camera, point);
			}

			const rapidjson::Value& jsonCameraCenter = jsonObservationObject["X"];
			float x = jsonCameraCenter[0].GetFloat(), y = jsonCameraCenter[1].GetFloat(), z = jsonCameraCenter[2].GetFloat();
			point->position = glm::vec3(x, y, z);
			// std::cout << "       idPoint: "<<point->idPoint << "\tidReconstruction: "<<point->idReconstruction << "\tgetNunmberObservation: "<<point->getNunmberObservation() << std::endl;

			if(conf_.fakePointsMultiplier) std::cerr << "conf_.fakePointsMultiplier option not available" << std::endl;

//			// FAKE POINTS
//			for(int fIndex = 0; fIndex < conf_.fakePointsMultiplier; fIndex++) {
//				long unsigned int fakePointId = (1+fIndex)*1000000 + jsonObservationObject["pointId"].GetInt();
//				PointType* fakePoint;
//
//				if (ORB_data_.hasPoint(fakePointId)) {
//					fakePoint = ORB_data_.getPoint(fakePointId);
//				} else {
//					fakePoint = new PointType();
//					fakePoint->idPoint = fakePointId;
//					ORB_data_.addPoint(fakePoint);
//				}
//
//				ORB_data_.addVisibility(camera, fakePoint);
//
////				const rapidjson::Value& jsonCameraCenter = jsonObservationObject["X"];
//				float fakeX = (1+fIndex)*0.1 + jsonCameraCenter[0].GetFloat(), fakeY = jsonCameraCenter[1].GetFloat(), fakeZ = jsonCameraCenter[2].GetFloat();
//				fakePoint->position = glm::vec3(fakeX, fakeY, fakeZ);
//			}

			//TODO point's 2D coordinates in frame
//      const rapidjson::Value& jsonCameraFrameCoordinates = jsonObservationObject["x"];
//      float u = jsonCameraFrameCoordinates[0].GetFloat(), v = jsonCameraFrameCoordinates[1].GetFloat();
//      ORB_data_.addFrameCoordinates(camera, point, glm::vec3(u, v));
		}

		return camera;

	} else {
		return NULL;
	}

}
//
//// TODO do a camera at a time
//void ORBIncrementalParser::parseViews() {
//  if(jsonViewIndex_ < jsonViewsArray_.Size()){
//    const rapidjson::Value& jsonView = jsonViewsArray_[jsonViewIndex_];
//    jsonViewIndex_++;
//
//    CameraType* camera = new CameraType();
//
//    camera->idCam = jsonView["key"].GetInt();
//    ORB_data_.addCamera(camera);
//
//    std::string local(jsonView["local_path"].GetString());
//    std::string filename(jsonView["filename"].GetString());
//    camera->pathImage = basePath + local + filename;
//
//    camera->imageWidth = jsonView["width"].GetInt();
//    camera->imageHeight = jsonView["height"].GetInt();
//
//    glm::mat3 rotation;
//    glm::vec3 center;
//    glm::vec3 translation;
//    glm::mat4 eMatrix(0.0), kMatrix(0.0);
//    glm::mat3 intrinsic = intrinsics_.at(jsonView["id_intrinsic"].GetInt());
//
//    for (int curR = 0; curR < 3; curR++) {
//      center[curR] = jsonView["extrinsic"]["center"][curR].GetFloat();
//    }
//
//    for(int curR = 0; curR < 3; curR++) for(int curC = 0; curC < 3; curC++){
//      kMatrix[curR][curC] = intrinsic[curR][curC];
//    }
//
//    for (int curR = 0; curR < 3; curR++) for (int curC = 0; curC < 3; curC++){
//      eMatrix[curR][curC] = jsonView["extrinsic"]["rotation"][curR][curC].GetFloat();
//      rotation[curR][curC] = jsonView["extrinsic"]["rotation"][curR][curC].GetFloat();
//    }
//
//    translation = -center * rotation;
//
//    eMatrix[0][3] = translation[0];
//    eMatrix[1][3] = translation[1];
//    eMatrix[2][3] = translation[2];
//    eMatrix[3][3] = 1.0;
//
//    camera->cameraMatrix =  eMatrix*kMatrix;
//    camera->rotation = glm::mat3(rotation);
//    camera->translation = glm::vec3(translation);
//    camera->center = glm::vec3(center);
//    camera->intrinsics =  intrinsic;
//    // TODO check camera properties are correct
//
//
//    const rapidjson::Value& jsonObservationsArray = jsonView["observations"];
//    for (rapidjson::SizeType jsonObservationIndex = 0; jsonObservationIndex < jsonObservationsArray.Size(); jsonObservationIndex++){
//      const rapidjson::Value& jsonObservationObject = jsonObservationsArray[jsonObservationIndex];
//
//      long unsigned int pointId = jsonObservationObject["key"].GetInt();
//      PointType* point;
//
//      if(ORB_data_.hasPoint(pointId)){
//        point = ORB_data_.getPoint(pointId);
//      }else{
//        point = new PointType();
//        point->idPoint = pointId;
//        ORB_data_.addPoint(point);
//      }
//
//      ORB_data_.addVisibility(camera, point);
//
//      const rapidjson::Value& jsonCameraCenter = jsonObservationObject["X"];
//      float x = jsonCameraCenter[0].GetFloat(), y = jsonCameraCenter[1].GetFloat(), z = jsonCameraCenter[2].GetFloat();
//      point->position = glm::vec3(x, y, z);
//
//      //TODO point's 2D coordinates in frame
////      const rapidjson::Value& jsonCameraFrameCoordinates = jsonObservationObject["x"];
////      float u = jsonCameraFrameCoordinates[0].GetFloat(), v = jsonCameraFrameCoordinates[1].GetFloat();
////      ORB_data_.addFrameCoordinates(camera, point, glm::vec3(u, v));
//    }
//  }
//}

void ORBIncrementalParser::parseIntrinsics() {

	try {

		if (!document_.HasMember("intrinsics")) throw JsonAccessException("JsonAccessException--> error while querying HasMember(views)");
		const rapidjson::Value& intrinsicsJson = document_["intrinsics"];

		for (rapidjson::SizeType curInt = 0; curInt < intrinsicsJson.Size(); curInt++) {
			int key = intrinsicsJson[curInt]["intrinsicId"].GetInt();
			glm::mat3 temp(0.0);
			temp[0][0] = intrinsicsJson[curInt]["value"]["ptr_wrapper"]["data"]["focal_length"].GetFloat();
			temp[1][1] = intrinsicsJson[curInt]["value"]["ptr_wrapper"]["data"]["focal_length"].GetFloat();
			temp[0][2] = intrinsicsJson[curInt]["value"]["ptr_wrapper"]["data"]["principal_point"][0].GetFloat();
			temp[1][2] = intrinsicsJson[curInt]["value"]["ptr_wrapper"]["data"]["principal_point"][1].GetFloat();
			temp[2][2] = 1.0;

			intrinsics_.insert(std::pair<int, glm::mat3>(key, temp));
		}

	} catch (JsonAccessException& e) {
		std::cerr << e.what() << std::endl;
		std::cout << e.what() << std::endl;

	}
}

std::string ORBIncrementalParser::getDataSPlot() {
	std::stringstream out;

	for (auto mCamera : ORB_data_.getCameras()) {
		CameraType* c = mCamera.second;
		long unsigned int idCam = c->idCam;
		for (auto p : c->visiblePointsT) {
			out << c->idCam COMMA p->position.x COMMA p->position.y COMMA p->position.z COMMA p->getNumberObservations() << std::endl;
		}

	}

	return out.str();
}

std::string ORBIncrementalParser::getDataCSV() {
	std::stringstream out;

	for (auto mCamera : ORB_data_.getCameras()) {
		CameraType* c = mCamera.second;
		glm::vec3 center = c->center;

		out << "c" COMMA c->idCam COMMA center.x COMMA center.y COMMA center.z << std::endl;

		for (auto p : c->visiblePointsT) {
			out << "p" COMMA p->idPoint COMMA c->idCam COMMA p->position.x COMMA p->position.y COMMA p->position.z << std::endl;
		}

	}

	return out.str();
}

std::string ORBIncrementalParser::getStats() {
	std::stringstream out;

	for (auto mCamera : ORB_data_.getCameras()) {
		CameraType* c = mCamera.second;

		out << "cam " << c->idCam << std::endl;
		std::map<int, int> count;

		for (auto p : c->visiblePointsT) {
			int nOcc = p->getNumberObservations();

			std::map<int, int>::iterator it = count.find(nOcc);
			if (it != count.end()) {
				count[nOcc] += 1;
			} else {
				count.insert(std::pair<int, int>(p->getNumberObservations(), 1));
			}

		}

		for (auto vkOcc : count) {
			int sum = 0;
			for (auto vkOcc_ : count) {
				if (vkOcc_.first >= vkOcc.first) sum += vkOcc_.second;
			}

			out << " =" << vkOcc.first << ":\t" << vkOcc.second << ";\t\t >=" << vkOcc.first << ":\t" << sum << std::endl;
		}
	}

	out << std::endl << std::endl;

	for (auto iCameraPair : ORB_data_.getCameras()) {
		CameraType* i = iCameraPair.second;
		std::set<long unsigned int> indexesSet;

		for (auto p : i->visiblePointsT)
			if (p->idReconstruction >= 0) for (auto j : p->viewingCams)
				indexesSet.insert(j->idCam);

		out << "co-cameras( " << i->idCam << "):\t";
		for (auto coCamIndex : indexesSet)
			out << " " << coCamIndex;
		out << std::endl;
	}

	float maxDistanceOverall = 0.0f, maxDistanceFromOOverall = 0.0f;
	for (auto cameraPair : ORB_data_.getCameras()) {
		float maxDistance = 0.0f, maxDistanceFromO = 0.0f;
		CameraType* c = cameraPair.second;
		for (auto p : c->visiblePointsT) {
			float d = utilities::distanceEucl(c->center.x, c->center.y, c->center.z, p->position.x, p->position.y, p->position.z);
			float d_o = utilities::distanceEucl(0.0f, 0.0f, 0.0f, p->position.x, p->position.y, p->position.z);
			if (d > maxDistance) maxDistance = d;
			if (d > maxDistanceOverall) maxDistanceOverall = d;
			if (d_o > maxDistanceFromO) maxDistanceFromO = d_o;
			if (d_o > maxDistanceFromOOverall) maxDistanceFromOOverall = d_o;
		}

		out << "camera " << c->idCam << "\tmax distance:\t" << maxDistance << "\tmax distance from O:\t" << maxDistanceFromO << std::endl;
	}

	out << "overall camera-points max distance:\t" << maxDistanceOverall << "\toverall max distance from O:\t" << maxDistanceFromOOverall << std::endl << std::endl;

	return out.str();

}

std::string ORBIncrementalParser::getDataOFF() {
	std::stringstream out;

	std::set<PointType*> visiblePoints;

	for (auto mCamera : ORB_data_.getCameras()) {
		CameraType* c = mCamera.second;

		for (auto p : c->visiblePointsT) {
			if (p->idReconstruction >= 0 && p->getNumberObservations() >= 2) visiblePoints.insert(p); // TODO if visibility is higher than 2
		}

	}

	out << "OFF" << std::endl << visiblePoints.size() << " 0 0" << std::endl;

	for (auto p : visiblePoints) {
		out << p->position.x SPACE p->position.y SPACE p->position.z << std::endl;
	}

	return out.str();
}

std::string ORBIncrementalParser::getPointsAsOFF(bool all, int minObservations) {
	std::stringstream out;
	int nPoints = 0;

	for (auto p : ORB_data_.getPoints())
		if ((all || p.second->idReconstruction >= 0) && p.second->getNumberObservations() >= minObservations) nPoints++;

	out << "OFF" << std::endl << nPoints << " 0 0" << std::endl;

	for (auto p : ORB_data_.getPoints())
		if ((all || p.second->idReconstruction >= 0) && p.second->getNumberObservations() >= minObservations)
			out << p.second->position.x SPACE p.second->position.y SPACE p.second->position.z << std::endl;

	return out.str();
}

void ORBIncrementalParser::ParseToOFF(std::string pathPrefix, int minObservationsRange) {

	for (int i = 0; i < numCameras(); i++)
		nextCamera();

	for (int minObservations = 1; minObservations <= minObservationsRange; minObservations++) {
		std::ofstream allPointsFile;
		std::ostringstream nameVisiblePoints;
		nameVisiblePoints << pathPrefix << minObservations << ".off";
		allPointsFile.open(nameVisiblePoints.str().c_str());
		allPointsFile << getPointsAsOFF(true, minObservations);
		allPointsFile.close();
	}

}
