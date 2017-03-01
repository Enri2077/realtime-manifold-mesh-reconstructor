/*
 * ReconstructFromSLAMData.h
 *
 *  Created on: 27 mar 2016
 *      Author: enrico
 */

#ifndef RECONSTRUCTFROMSLAMDATA_H_
#define RECONSTRUCTFROMSLAMDATA_H_

#include <TriangulationManager.h>
#include <types_config.hpp>
#include <OutputManager.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Chronometer.h>

class ReconstructFromSLAMData {
public:
	ReconstructFromSLAMData(ManifoldReconstructionConfig& config);
	virtual ~ReconstructFromSLAMData();

	void addCamera(CameraType* newCamera);
	void update();
	void saveMesh(std::string namePrefix, std::string nameSuffix);
	bool integrityCheck();

	void overwriteFocalY(float f);

	void setExpectedTotalIterationsNumber(int n);
	void insertStatValue(float v);


	OutputManager* getOutputManager() {
		return manifRec_->getOutputManager();
	}

	// Iteration counter. Incremented every time a camera is added
	int iterationCount;

private:

	ManifoldReconstructionConfig& config_;

	TriangulationManager* manifRec_;

//	std::set<CameraType*> rayTracingSet_;

	// Camera's and point's index used in ManifoldMeshReconstructor must be set incrementally every time the camera or point is effectivly added to ManifoldMeshReconstructor
	int cameraNextId, pointNextId;

	// Flag representing whether the manifold was updated since the last time it was saved. Set to true by updateManifold, to false by saveManifold
	bool triangulationUpdatedSinceSave_;

	int expectedTotalIterationsNumber_ = 0;
};

#endif /* RECONSTRUCTFROMSLAMDATA_H_ */
