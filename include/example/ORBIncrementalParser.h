
#ifndef CAM_PARSERS_ORBINCREMENTALPARSER_H_
#define CAM_PARSERS_ORBINCREMENTALPARSER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <rapidjson/document.h>
#include <CameraPointsCollection.h>
#include <types_reconstructor.hpp>
#include <types_config.hpp>

class ORBIncrementalParser {
public:
	ORBIncrementalParser(std::string path, ManifoldReconstructionConfig conf);
	virtual ~ORBIncrementalParser();

	int numCameras();
	CameraType* nextCamera();

	std::string getStats();
	std::string getDataCSV();
	std::string getDataSPlot();
	std::string getDataOFF();

	std::string getPointsAsOFF(bool all, int minObservations);
	void ParseToOFF(std::string pathPrefix, int minObservationsRange);

	const CameraPointsCollection& getData() const {
		return ORB_data_;
	}

private:
	void parseViews();
	void parseIntrinsics();

	rapidjson::Document document_;
	rapidjson::Value jsonViewsArray_;
	rapidjson::SizeType jsonViewIndex_;
	std::ifstream fileStream_;
	std::string fileName_;

	std::string basePath_;
	std::map<int, glm::mat3> intrinsics_;
	CameraPointsCollection ORB_data_;
	ManifoldReconstructionConfig conf_;

	bool insertFakePoints_ = true;

};

#endif /* CAM_PARSERS_ORBINCREMENTALPARSER_H_ */
