/*
 * ReconstructFromSLAMData.cpp
 *
 *  Created on: 27 mar 2016
 *      Author: enrico
 */

#include <CameraPointsCollection.h>
#include <ReconstructFromSLAMData.h>
#include <utilities.hpp>

#define VERBOSE_UPDATE_MANIFOLD_FUNCTION false
#define VERBOSE_ADD_CAMERA_FUNCTION false
#define VERBOSE_SAVE_MANIFOLD_FUNCTION false
#define VERBOSE_CAMERA_ADD false
#define VERBOSE_CAMERA_UPDATE true
#define VERBOSE_POINT_ADD false
#define VERBOSE_POINT_UPDATE false
#define VERBOSE_POINT_IGNORE false
#define VERBOSE_ADD_VISIBILITY_PAIR false
#define VERBOSE_POINTS_COUNT false

ReconstructFromSLAMData::ReconstructFromSLAMData(ManifoldReconstructionConfig& config) :
		config_(config) {
	iterationCount = 0;

	manifRec_ = new TriangulationManager(config_);

	cameraNextId = 0;
	pointNextId = 0;

	// Initially the triangulation is empty, so it is considered not to be saved until the first update
	triangulationUpdatedSinceSave_ = false;

}

ReconstructFromSLAMData::~ReconstructFromSLAMData() {
}

void ReconstructFromSLAMData::setExpectedTotalIterationsNumber(int n) {
	expectedTotalIterationsNumber_ = n;
}

void ReconstructFromSLAMData::addCamera(CameraType* newCamera) {
	if (config_.timeStatsOutput && VERBOSE_ADD_CAMERA_FUNCTION) {
		std::cout << std::endl << "ReconstructFromSLAMData::addCamera iteration " << iterationCount << " / " << expectedTotalIterationsNumber_ - 1 << std::endl;
	}

	bool isCameraNew;

	if (newCamera->idReconstruction < 0) {

		// Add the new camera to ManifoldMeshReconstructor
		glm::vec3 center = newCamera->center;
		manifRec_->addCamera(center.x, center.y, center.z);

		// Generate the ManifoldMeshReconstructor's index if the camera doesn't have it already
		isCameraNew = true;
		newCamera->idReconstruction = cameraNextId++;

		if (config_.timeStatsOutput && VERBOSE_CAMERA_ADD) {
			std::cout << "ADD cam " << newCamera->idCam << " (" << newCamera->idReconstruction << ")" << ": " << center.x << ", " << center.y << ", " << center.z << std::endl;
		}

	} else {
		// If the camera already has the ManifoldMeshReconstructor's index, then it is updated
		isCameraNew = false;

		glm::vec3 center = newCamera->center;
		manifRec_->moveCamera(newCamera->idReconstruction, center.x, center.y, center.z);

		if (config_.timeStatsOutput && VERBOSE_CAMERA_UPDATE) {
			std::cout << "UPDATE cam " << newCamera->idCam << " (" << newCamera->idReconstruction << ")" << ": " << center.x << ", " << center.y << ", " << center.z << std::endl;
		}
	}

	// Add the points associated to the new camera to ManifoldMeshReconstructor
	int countIgnoredPoints = 0, countUpdatedPoints = 0, countAddedPoints = 0, countPairedPoints = 0;
	for (auto const &p : newCamera->visiblePointsT) {

		// Only consider points that were observed in many frames
		if (p->getNumberObservations() < config_.primaryPointsVisibilityThreshold) {
			countIgnoredPoints++;
			if (config_.timeStatsOutput && VERBOSE_POINT_IGNORE) {
				std::cout << "IGNORE point \t" << p->idPoint << "\t (recId:" << p->idReconstruction << "),\tobs: " << p->getNumberObservations() << std::endl;
			}
			continue;
		}

		if (p->idReconstruction < 0) {
			// Generate the ManifoldMeshReconstructor's index if the point doesn't have it already
			p->idReconstruction = pointNextId++;

			// If the point didn't have the idReconstruction index, then it is a new point for sure
			glm::vec3 position = p->position;
			manifRec_->addPoint(position.x, position.y, position.z, p->r, p->g, p->b, p->a);
			countAddedPoints++;

			// Since the point has just been added, the visibility with the previous (and current) cameras wasn't added before. Add it now
			for (auto coCamera : p->viewingCams) {
				if (coCamera->idReconstruction >= 0) {
					manifRec_->addVisibilityPair(coCamera->idReconstruction, p->idReconstruction);

					countPairedPoints++;
					if (config_.timeStatsOutput && VERBOSE_ADD_VISIBILITY_PAIR) {
						std::cout << "add visibility with co-camera " << coCamera->idCam << ", point " << p->idPoint << std::endl;
					}
				} else {
					// All the previous cameras should have been added already.
					// This shouldn't happen, unless some cameras are in CameraPointsCollection but weren't added by ReconstructFromSLAMData::addCamera (this function)
					//std::cerr << "camera " << coCamera->idCam << " ignored" << std::endl;
				}
			}

			if (config_.timeStatsOutput && VERBOSE_POINT_ADD) {
				std::cout << "ADD    point \t" << p->idPoint << "\t (recId:" << p->idReconstruction << "),\tobs: " << p->getNumberObservations() << ": " << position.x << ", " << position.y << ", " << position.z << std::endl;
			}

		} else {

			// The point was already added to ManifoldMeshReconstructor, so it is only updated
			glm::vec3 position = p->position;
			manifRec_->movePoint(p->idReconstruction, position.x, position.y, position.z, p->r, p->g, p->b, p->a);
			countUpdatedPoints++;
			if (config_.timeStatsOutput && VERBOSE_POINT_UPDATE) {
				std::cout << "UPDATE point \t" << p->idPoint << "\t (recId:" << p->idReconstruction << "),\tobs: " << p->getNumberObservations() << ": " << position.x << ", " << position.y << ", " << position.z << std::endl;
			}

			if (isCameraNew) {
				// Add visibility between the (already added) point and the just added camera
				manifRec_->addVisibilityPair(newCamera->idReconstruction, p->idReconstruction);

				countPairedPoints++;
				if (config_.timeStatsOutput && VERBOSE_ADD_VISIBILITY_PAIR) {
					std::cout << "add visibility with new camera " << newCamera->idCam << ", point " << p->idPoint << std::endl;
				}
			} //else: If the camera is being updated, the visibility pair was already added
		}
	}

	if (config_.timeStatsOutput && VERBOSE_POINTS_COUNT) {
		std::cout << "Added   Points:   " << countAddedPoints << std::endl << "Updated Points:   " << countUpdatedPoints << std::endl << "Ignored Points:   " << countIgnoredPoints << std::endl << "Paired  Points:   " << countPairedPoints << std::endl << std::endl;
	}

	iterationCount++;
}

void ReconstructFromSLAMData::update() {
	if (VERBOSE_UPDATE_MANIFOLD_FUNCTION) {
		std::cout << std::endl << "ReconstructFromSLAMData::updateManifold iteration " << iterationCount << " / " << expectedTotalIterationsNumber_ - 1 << std::endl;
	}

	triangulationUpdatedSinceSave_ = true;

	//TODO Actually manage the case of newCameras have too few or no points associated (in ManifoldMeshReconstructior) : count( newCamera.points st idRec>=0 )

	// Tell ManifoldMeshReconstructor to update the triangulation with the new cameras and points
	Chronometer chronoUpdate;
	chronoUpdate.start();

	manifRec_->updateTriangulation();

	chronoUpdate.stop();
	std::cout << "updateTriangulation\t\t" << chronoUpdate.getSeconds() << std::endl;

}

void ReconstructFromSLAMData::saveMesh(std::string namePrefix, std::string nameSuffix) {
	if (config_.timeStatsOutput && VERBOSE_SAVE_MANIFOLD_FUNCTION) {
		std::cout << std::endl << "ReconstructFromSLAMData::saveManifold iteration " << iterationCount << " / " << expectedTotalIterationsNumber_ - 1 << std::endl;
	}

	if (triangulationUpdatedSinceSave_ == false) return;
	triangulationUpdatedSinceSave_ = false;

	Chronometer chronoSave;
	chronoSave.start();

	std::ostringstream fileName;
	fileName << namePrefix << "manifold_" << nameSuffix << ".off";
	std::cout << "saving " << fileName.str() << std::endl;
	manifRec_->saveManifold(fileName.str());

	chronoSave.stop();
	if (config_.timeStatsOutput) std::cout << "save manifold\t\t\t" << chronoSave.getSeconds() << std::endl;
}

bool ReconstructFromSLAMData::integrityCheck() {
	return manifRec_->integrityCheck();
}

void ReconstructFromSLAMData::insertStatValue(float v) {
	manifRec_->timeStatsFile_ << v << ", ";
}
