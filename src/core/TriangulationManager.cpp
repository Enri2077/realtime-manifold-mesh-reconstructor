/*
 * TriangulationManager.cpp
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#include <TriangulationManager.h>
#include <vector>
#include <utilities.hpp>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <algorithm>
#include <math.h>
#include <omp.h>

using std::cout;
using std::cerr;
using std::endl;
using std::pair;

TriangulationManager::TriangulationManager(ManifoldReconstructionConfig& conf) :
		conf_(conf) {

	manifoldManager_ = new ManifoldManager(dt_, conf_);

	manifoldManager_->getOutputManager()->setPoints(&points_);

	stepX_ = stepY_ = stepZ_ = conf_.steinerGridStepLength;

	sgMinX_ = sgMinY_ = sgMinZ_ = sgMaxX_ = sgMaxY_ = sgMaxZ_ = 0.0;

	sgCurrentMinX_ = sgCurrentMinY_ = sgCurrentMinZ_ = 0.0;
	sgCurrentMaxX_ = sgCurrentMaxY_ = sgCurrentMaxZ_ = conf_.steinerGridStepLength;

	std::stringstream statsFileName;
	statsFileName << conf_.timeStatsFolder << "timeStats " << conf_.statsId << ".csv";
	cout << "Opening stats file " << statsFileName.str() << endl;
	timeStatsFile_.open(statsFileName.str());
	timeStatsFile_ << "cameras number, updateSteinerGrid, rayRemoving, shrinkManifold, shrinkSingle, shrinkSeveral, Remove vertices, Remove points, Add new vertices, Move vertices, Move Cameras, rayUntracing, rayTracing, rayRetracing, growManifold, growManifoldSev, growManifold, InsertInBoundary, RemoveFromBoundary, isRegular, singleTest, addedPoints, movedPoints, Overall" << endl;

	std::stringstream countsFileName;
	countsFileName << conf_.countStatsFolder << "countStats " << conf_.statsId << ".csv";
	cout << "Opening counts file " << countsFileName.str() << endl;
	countStatsFile_.open(countsFileName.str());
	countStatsFile_ << "Iteration, Updated Cameras, Added Points, Moved Points, Inserted Steiner Points, Tracing Index Cache Hit Ratio, Traced Rays, Retraced Rays, Untraced Rays, Insert In Boundary, Remove From Boundary, Single Cell Test, Is Regular Test, Enclosing Cells, Enclosing Volume Cache Hit, Shrinked Singular, Shrinked Several, Grown Singular, Grown Several" << endl;

}

TriangulationManager::~TriangulationManager() {
	delete (manifoldManager_);
	timeStatsFile_.close();
	countStatsFile_.close();
}

void TriangulationManager::updateTriangulation() {

	timeStatsFile_ << endl << cameras_.size() << ", ";

	rt2_ChronoUseless_.reset();
	rt2_ChronoFirstCell_.reset();
	rt2_ChronoCellTraversing_.reset();
	rt2_ChronoNeighboursD1Selection_.reset();
	rt2_ChronoNeighboursD2Selection_.reset();
	rt2_ChronoNeighboursD1WeightUpdate_.reset();
	rt2_ChronoNeighboursD2WeightUpdate_.reset();

	manifoldManager_->chronoInsertInBoundary_.reset();
	manifoldManager_->chronoRemoveFromBoundary_.reset();
	manifoldManager_->chronoIsRegularOverall_.reset();
	manifoldManager_->chronoIsRegularOverall2_.reset();
	manifoldManager_->chronoSingleTestOverall_.reset();

	rt2_CountNeighboursD1WeightUpdate_ = 0;
	rt2_CountNeighboursD2WeightUpdate_ = 0;

	rt2_SuccessfulCachedIndices = 0;
	rt2_TriedCachedIndices = 0;

	manifoldManager_->countInsertInBoundary_ = 0;
	manifoldManager_->countRemoveFromBoundary_ = 0;
	manifoldManager_->countSingleTestOverall_ = 0;
	manifoldManager_->countIsRegularTest_ = 0;
	manifoldManager_->countEnclosingCells_ = 0;
	manifoldManager_->countEnclosingVolumeCacheHit_ = 0;
	manifoldManager_->countEnclosingVolumeCacheTotal_ = 0;
	manifoldManager_->countShrinkedSingular_ = 0;
	manifoldManager_->countShrinkedSeveral_ = 0;
	manifoldManager_->countGrownSingular_ = 0;
	manifoldManager_->countGrownSeveral_ = 0;

	countTracedRays_ = 0;
	countRetracedRays_ = 0;
	countUntracedRays_ = 0;
	countInsertedSteinerPoints_ = 0;

	Chronometer chronoCheck, chronoEverything;
	chronoCheck.start();
	chronoCheck.stop();
	if (conf_.timeStatsOutput) cout << "TriangulationManager::updateTriangulation: \t\t\t min chrono time\t\t\t" << chronoCheck.getNanoseconds() << " ns" << endl;

	int addedPointsStat = 0, movedPointsStat = 0, updatedCamerasStat = updatedCamerasId_.size();

	/*
	 *  Steiner grid updating
	 */

	if (dt_.number_of_vertices() == 0) {
		chronoEverything.reset();
		chronoEverything.start();

		initSteinerPointGridAndBound();
		updateSteinerPointGridAndBound();

		chronoEverything.stop();
		if (conf_.timeStatsOutput) cout << "initSteinerGrid\t\t" << chronoEverything.getSeconds() << endl << endl;
	} else {
		chronoEverything.reset();
		chronoEverything.start();

		updateSteinerPointGridAndBound();

		chronoEverything.stop();
		if (conf_.timeStatsOutput) cout << "updateSteinerGrid\t\t" << chronoEverything.getSeconds() << endl << endl;
	}
	timeStatsFile_ << chronoEverything.getSeconds() << ", ";

	/*
	 *  Ray removing
	 */

	int pointsRemovedCount = -1, verticesRemovedCount = -1, raysRemovedAndInvalidatedCount = 0, raysInvaldatedCount = 0, raysRemovedCount = 0,
			raysCandidateToBeRemovedCount = raysCandidateToBeRemoved_.size();

	if (raysCandidateToBeRemoved_.size()) {

//		std::vector<Segment> outputRays;
//		for (auto cIndex_pIndex : raysCandidateToBeRemoved_) {
//			RayPath* r = getRayPath(cIndex_pIndex.first, cIndex_pIndex.second);
//			if (r->mistrustVote > conf_.rayRemovalThreshold) outputRays.push_back(Segment(cams_[r->cameraId].position, points_[r->pointId].position));
//		}
//		manifoldManager_->getOutputManager()->writeRaysToOFF("output/erasedRays/", std::vector<int> { iterationCounter_ }, outputRays);

		chronoEverything.reset();
		chronoEverything.start();
		for (auto cIndex_pIndex : raysCandidateToBeRemoved_) {
			RayPath* r = getRayPath(cIndex_pIndex.first, cIndex_pIndex.second);

			if (r->mistrustVote > conf_.rayRemovalThreshold) {
				raysRemovedCount++;
				removeRay(r);
			}
		}
		raysCandidateToBeRemoved_.clear();

		chronoEverything.stop();
		if (conf_.timeStatsOutput) cout << "rayRemoving\t\t" << chronoEverything.getSeconds() << endl << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.timeStatsOutput) cout << "rayRemoving\t\tSkipped" << endl << endl;
		timeStatsFile_ << 0.0 << ", ";
	}
	if (conf_.timeStatsOutput) cout << "TriangulationManager::updateTriangulation:\t\t rays removed:\t\t\t\t" << raysRemovedCount << "\t /\t" << raysCandidateToBeRemovedCount << endl;

	/*
	 * 	Vertex removing based on min adjacent freeVote
	 */

	if (conf_.enableUnusedVertexRemoving) {
		for (auto v = dt_.finite_vertices_begin(); v != dt_.finite_vertices_end(); v++) {
			std::vector<Delaunay3::Cell_handle> incidentCells;
			bool canBeRemoved = true;
			dt_.incident_cells(v, std::back_inserter(incidentCells));

			if (v->info().getPointId() == -1) continue; // the vertex is associated with a Steiner point and should not be removed

			for (auto c : incidentCells)
				if (!c->info().getManifoldFlag() || c->info().getFreeVote() <= conf_.unusedVertexRemovalThreshold) {
					canBeRemoved = false;
					break;
				}

			if (canBeRemoved) verticesToBeRemoved_.push_back(v);
		}

		verticesRemovedCount = verticesToBeRemoved_.size();
	}

	/*
	 *  Enclosing
	 */

	// TODO also include the points that will be removed
	std::set<PointD3> enclosingVolumePoints;

	for (auto pIndex : pointsToBeRemovedId_) {
		enclosingVolumePoints.insert(points_[pIndex].position);
	}

	for (auto v : verticesToBeRemoved_) {
		int pointId = v->info().getPointId(); // TODO type
		if (pointId >= 0) enclosingVolumePoints.insert(points_[pointId].position);
	}

	if (conf_.enablePointsPositionUpdate) for (auto pIndex : movedPointsId_) {
		if(utilities::distanceEucl(points_[pIndex].position, points_[pIndex].newPosition) > conf_.minDistancePointPositionUpdate)
			enclosingVolumePoints.insert(points_[pIndex].newPosition);
	}

	for (auto cIndex : updatedCamerasId_)
		for (auto pIndex : cameras_[cIndex].newVisiblePoints)
			if (points_[pIndex].notTriangulated && utilities::distanceEucl(points_[pIndex].position, cameras_[cIndex].position) < conf_.maxDistanceCameraPoints)
				enclosingVolumePoints.insert(points_[pIndex].position);

	// TODO use cubed sphere to decrease the number of cells that will be tested
	int subMapBounds = 2;
	std::set<index3> enclosingVolumeMapIndices;
	for (auto p : enclosingVolumePoints) {
		// The index of the grid's cube is the integer rounded down for each coordinate
		int i = std::floor(p.x() / conf_.steinerGridStepLength);
		int j = std::floor(p.y() / conf_.steinerGridStepLength);
		int k = std::floor(p.z() / conf_.steinerGridStepLength);

		for (int i_ = -subMapBounds; i_ <= subMapBounds; i_++)
			for (int j_ = -subMapBounds; j_ <= subMapBounds; j_++)
				for (int k_ = -subMapBounds; k_ <= subMapBounds; k_++)
					enclosingVolumeMapIndices.insert(index3(i + i_, j + j_, k + k_));
	}

	// This is used to cache the enclosing information in the cells, incrementing it invalidates the cached values and needs to be done when the points on which the enclosing volume is based are changed
	currentEnclosingVersion_++;

//	if (conf_.debugOutput) manifoldManager_->checkBoundaryIntegrity();

	/*
	 *  Shrinking
	 */

	timerShrinkTime_ = 0.0;
	timerShrinkSeveralTime_ = 0.0;
	if (enclosingVolumePoints.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		shrinkManifold(enclosingVolumeMapIndices);

		chronoEverything.stop();
		if (conf_.timeStatsOutput) cout << "Overall shrink\t\t" << chronoEverything.getSeconds() << endl << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.timeStatsOutput) cout << "Overall shrink\t\tSkipped" << endl << endl;
		timeStatsFile_ << 0.0 << ", ";
	}
	timeStatsFile_ << timerShrinkTime_ << ", " << timerShrinkSeveralTime_ << ", ";

//	if (conf_.debugOutput) manifoldManager_->checkBoundaryIntegrity();

	/*
	 *  Vertex removing
	 */

	if (verticesToBeRemoved_.size()) {

		std::vector<Segment> outputRays;
		for (auto v : verticesToBeRemoved_) {
			auto pointId = v->info().getPointId();
			auto pointPosition = points_[pointId].position;

			for (auto cameraId : points_[pointId].viewingCams)
				outputRays.push_back(Segment(cameras_[cameraId].position, pointPosition));

		}
		manifoldManager_->getOutputManager()->writeRaysToOFF("output/erasedRays/", std::vector<int> { iterationCounter_ }, outputRays);

		chronoEverything.reset();
		chronoEverything.start();

		for (auto v : verticesToBeRemoved_) {
			auto pointId = v->info().getPointId();

			for (auto cameraId : points_[pointId].viewingCams)
				removeRay(getRayPath(cameraId, pointId));

			removeVertex(pointId);
		}
		verticesToBeRemoved_.clear();

		chronoEverything.stop();
		if (conf_.timeStatsOutput) cout << "Remove vertices\t\t" << chronoEverything.getSeconds() << endl << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.timeStatsOutput) cout << "Remove vertices\t\tSkipped" << endl << endl;
		timeStatsFile_ << 0.0 << ", ";
	}
	if (conf_.timeStatsOutput) cout << "TriangulationManager::rayTracingFromAllCam:\t\t vertices removed:\t " << verticesRemovedCount << endl;

	/*
	 *  Points removing
	 */
	pointsRemovedCount = pointsToBeRemovedId_.size();

	if (pointsToBeRemovedId_.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		for (auto id : pointsToBeRemovedId_) {
			if (!points_[id].notTriangulated) removeVertex(id);
		}
		pointsToBeRemovedId_.clear();

		chronoEverything.stop();
		if (conf_.timeStatsOutput) cout << "Remove points\t\t" << chronoEverything.getSeconds() << endl << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.timeStatsOutput) cout << "Remove points\t\tSkipped" << endl << endl;
		timeStatsFile_ << 0.0 << ", ";
	}
	if (conf_.timeStatsOutput) cout << "TriangulationManager::rayTracingFromAllCam:\t\t points removed:\t " << pointsRemovedCount << endl;

	/*
	 *  Vertex inserting
	 */

	if (updatedCamerasId_.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		for (auto updatedCameraIndex : updatedCamerasId_) {

			for (auto pIndex : cameras_[updatedCameraIndex].newVisiblePoints) {
				PointReconstruction& point = points_[pIndex];

				if (point.notTriangulated) {

					if (utilities::distanceEucl(point.position, cameras_[updatedCameraIndex].position) < conf_.maxDistanceCameraPoints) {

						/*	Try to insert the new point in the triangulation.
						 * 	When successful, a new vertex corresponding to the new point is created,
						 * 	some cells are removed from the triangulation and replaced by some others.
						 */
						if (insertVertex(point)) {
							addedPointsStat++;
						}
					}

				} else {
					raysToBeTraced_.insert(pair<int, int>(updatedCameraIndex, point.idReconstruction));
				}
			}

			cameras_[updatedCameraIndex].newVisiblePoints.clear();

			/*
			 *  Camera Steiner Point inserting
			 *  (Inserts a Steiner point near every camera to avoid visual artifacts on the path followed by the camera)
			 *  (turns out it creates Steiner artifacts in exchange for normal artifacts: not worth it until the Steiner artifacts can be automatically removed)
			 */

//			PointReconstruction cameraSteinerPoint;
//
//			cameraSteinerPoint.idReconstruction = nextSteinerPointId_--;
//			PointD3 position = PointD3(cameras_[updatedCameraIndex].position.x() + 0.01, cameras_[updatedCameraIndex].position.y() + 0.01, cameras_[updatedCameraIndex].position.z() + 0.01);
//
//			cameraSteinerPoint.position = position;
//			cameraSteinerPoint.notTriangulated = true;
//			cameraSteinerPoint.toBeMoved = false;
//
//			insertVertex(cameraSteinerPoint);

		}

		updatedCamerasId_.clear();

		chronoEverything.stop();
		if (conf_.timeStatsOutput) cout << "Add new vertices\t" << chronoEverything.getSeconds() << endl << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.timeStatsOutput) cout << "Add new vertices\tSkipped" << endl << endl;
		timeStatsFile_ << 0.0 << ", ";
	}





	/*
	 *  Vertex moving
	 */

	int totalMovedPointsStat = movedPointsId_.size();

	if (conf_.enablePointsPositionUpdate && movedPointsId_.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		for (auto id : movedPointsId_) {

			Segment s = Segment(points_[id].position, points_[id].newPosition);

			bool moved;
			moved = moveVertex(id);

			if (moved) movedPointsStat++;

			if (conf_.debugOutput) if (moved) movedPointsSegments_.push_back(s);

		}
		movedPointsId_.clear();

		chronoEverything.stop();
		if (conf_.timeStatsOutput) cout << "Move vertices\t\t" << chronoEverything.getSeconds() << endl << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.timeStatsOutput) cout << "Move vertices\t\tSkipped" << endl << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

	if (conf_.debugOutput) manifoldManager_->getOutputManager()->writeRaysToOFF("output/moved_points/moved_points", std::vector<int> { }, movedPointsSegments_);

	/*
	 *  Camera moving
	 */

	int movedCamerasStat = 0, totalMovedCamerasStat = movedCamerasId_.size();

	if (movedCamerasId_.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		for (int cameraIndex : movedCamerasId_) {
			bool moved;
			moved = moveCameraConstraints(cameraIndex);

			if (moved) movedCamerasStat++;

		}
		movedCamerasId_.clear();

		chronoEverything.stop();
		if (conf_.timeStatsOutput) cout << "Move Cameras\t\t" << chronoEverything.getSeconds() << endl << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.timeStatsOutput) cout << "Move Cameras\t\tSkipped" << endl << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

	/*
	 *  Ray untracing
	 *  Ray tracing
	 *  Ray retracing
	 */

	rayTracingFromAllCam();

//	if (conf_.debugOutput) manifoldManager_->checkBoundaryIntegrity();

	/*
	 *  Grow
	 */

	growManifold(enclosingVolumeMapIndices);

//	if (conf_.debugOutput) manifoldManager_->checkBoundaryIntegrity();

	timeStatsFile_ << manifoldManager_->chronoInsertInBoundary_.getSeconds() << ", ";
	timeStatsFile_ << manifoldManager_->chronoRemoveFromBoundary_.getSeconds() << ", ";

	timeStatsFile_ << manifoldManager_->chronoIsRegularOverall_.getSeconds() + manifoldManager_->chronoIsRegularOverall2_.getSeconds() << ", ";
	timeStatsFile_ << manifoldManager_->chronoSingleTestOverall_.getSeconds() << ", ";

	timeStatsFile_ << (float) addedPointsStat / 100 / updatedCamerasStat << ", ";
	timeStatsFile_ << (float) movedPointsStat / 100 / updatedCamerasStat << ", ";

	countStatsFile_ << iterationCounter_ << ", ";
	countStatsFile_ << updatedCamerasStat << ", ";
	countStatsFile_ << addedPointsStat << ", ";
	countStatsFile_ << movedPointsStat << ", ";
	countStatsFile_ << countInsertedSteinerPoints_ << ", ";
	countStatsFile_ << (double) rt2_SuccessfulCachedIndices / (double) rt2_TriedCachedIndices << ", ";
	countStatsFile_ << countTracedRays_ << ", ";
	countStatsFile_ << countRetracedRays_ << ", ";
	countStatsFile_ << countUntracedRays_ << ", ";

	countStatsFile_ << manifoldManager_->countInsertInBoundary_ << ", ";
	countStatsFile_ << manifoldManager_->countRemoveFromBoundary_ << ", ";
	countStatsFile_ << manifoldManager_->countSingleTestOverall_ << ", ";
	countStatsFile_ << manifoldManager_->countIsRegularTest_ << ", ";
	countStatsFile_ << manifoldManager_->countEnclosingCells_ << ", ";
	countStatsFile_ << (double) manifoldManager_->countEnclosingVolumeCacheHit_ / (double) manifoldManager_->countEnclosingVolumeCacheTotal_ << ", ";
	countStatsFile_ << manifoldManager_->countShrinkedSingular_ << ", ";
	countStatsFile_ << manifoldManager_->countShrinkedSeveral_ << ", ";
	countStatsFile_ << manifoldManager_->countGrownSingular_ << ", ";
	countStatsFile_ << manifoldManager_->countGrownSeveral_ << ", ";
	countStatsFile_ << endl;

	if (conf_.timeStatsOutput) {
		cout << "TriangulationManager::updateTriangulation:\t\t\t mm \t updated cameras:\t\t" << updatedCamerasStat << endl;
		cout << "TriangulationManager::updateTriangulation:\t\t\t mm \t added points:\t\t\t" << addedPointsStat << endl;
		cout << "TriangulationManager::updateTriangulation:\t\t\t mm \t moved points:\t\t\t" << movedPointsStat << "\t/\t" << totalMovedPointsStat << endl;
		cout << "TriangulationManager::updateTriangulation:\t\t\t mm \t moved cameras:\t\t\t" << movedCamerasStat << "\t/\t" << totalMovedCamerasStat << endl;

		cout << "TriangulationManager::updateTriangulation:\t\t\t mm \t insertInBoundary:\t\t" << manifoldManager_->chronoInsertInBoundary_.getSeconds() << " s" << endl;
		cout << "TriangulationManager::updateTriangulation:\t\t\t mm \t removeFromBoundary:\t\t" << manifoldManager_->chronoRemoveFromBoundary_.getSeconds() << " s" << endl;

		cout << "TriangulationManager::updateTriangulation:\t\t\t mm \t isRegularTest/1:\t\t\t" << manifoldManager_->chronoIsRegularOverall_.getSeconds() << " s" << endl;
		cout << "TriangulationManager::updateTriangulation:\t\t\t mm \t isRegularTest/2:\t\t\t" << manifoldManager_->chronoIsRegularOverall2_.getSeconds() << " s" << endl;
		cout << "TriangulationManager::updateTriangulation:\t\t\t mm \t singleTest:\t\t\t" << manifoldManager_->chronoSingleTestOverall_.getSeconds() << " s" << endl;

		cout << "TriangulationManager::updateTriangulation:\t\t\t rt2\t useless:\t\t\t" << rt2_ChronoUseless_.getSeconds() << " s" << endl;
		cout << "TriangulationManager::updateTriangulation:\t\t\t rt2\t first cell:\t\t\t" << rt2_ChronoFirstCell_.getSeconds() << " s" << endl;
		cout << "TriangulationManager::updateTriangulation:\t\t\t rt2\t cell traversing:\t\t" << rt2_ChronoCellTraversing_.getSeconds() << " s" << endl;
		cout << "TriangulationManager::updateTriangulation:\t\t\t rt2\t index cache hit ratio:\t\t" << (double) rt2_SuccessfulCachedIndices / (double) rt2_TriedCachedIndices << endl;
		cout << "TriangulationManager::updateTriangulation:\t\t\t rt2\t neighbours weight update:\t" << rt2_ChronoNeighboursD1WeightUpdate_.getSeconds() << " s\t / \t" << rt2_CountNeighboursD1WeightUpdate_ << endl;
		cout << "TriangulationManager::updateTriangulation:\t\t\t rt2\t neighbours int insert/remove:\t" << rt2_ChronoNeighboursD2WeightUpdate_.getSeconds() << " s\t / \t" << rt2_CountNeighboursD2WeightUpdate_ << endl;
		cout << endl;
	}

	iterationCounter_++;
}

void TriangulationManager::addPoint(float x, float y, float z) {
	PointReconstruction t;
	t.idReconstruction = points_.size();
	t.position = PointD3(x, y, z);
	points_.push_back(t);
}

void TriangulationManager::addPoint(float x, float y, float z, float r, float g, float b, float a) {
	PointReconstruction t;
	t.idReconstruction = points_.size();
	t.position = PointD3(x, y, z);
	t.r = r;
	t.g = g;
	t.b = b;
	t.a = a;

	points_.push_back(t);
}

void TriangulationManager::movePoint(int id, float x, float y, float z) {
	points_[id].newPosition = PointD3(x, y, z);
	points_[id].toBeMoved = true;
	movedPointsId_.push_back(id);
}

void TriangulationManager::movePoint(int id, float x, float y, float z, float r, float g, float b, float a) {

	points_[id].newPosition = PointD3(x, y, z);
	points_[id].toBeMoved = true;
	movedPointsId_.push_back(id);

	points_[id].r = r;
	points_[id].g = g;
	points_[id].b = b;
	points_[id].a = a;
}

void TriangulationManager::addCamera(float x, float y, float z) {
	CamReconstruction t;
	t.idReconstruction = cameras_.size();
	t.position = PointD3(x, y, z);
	glm::vec3 pos = glm::vec3(x, y, z);

	cameras_.push_back(t);

	updatedCamerasId_.insert(t.idReconstruction);

}

void TriangulationManager::moveCamera(int id, float x, float y, float z) {
	cameras_[id].newPosition = PointD3(x, y, z);
	cameras_[id].toBeMoved = true;

	if (conf_.timeStatsOutput) {
		std::cout << "TriangulationManager::moveCamera camera " << id << "\t D: " << utilities::distanceEucl(cameras_[id].newPosition, cameras_[id].position) << std::endl;
	}

	movedCamerasId_.push_back(id);
	updatedCamerasId_.insert(id);

	updateSteinerGridTargetBounds(x, y, z);
}

void TriangulationManager::addVisibilityPair(int cameraId, int pointId) {
	CamReconstruction& c = cameras_[cameraId];
	PointReconstruction& p = points_[pointId];

	c.visiblePoints.push_back(pointId);
	c.newVisiblePoints.push_back(pointId);
	p.viewingCams.push_back(cameraId);

	if (utilities::distanceEucl(c.position, p.position) < conf_.maxDistanceCameraPoints) {
		updateSteinerGridTargetBounds(c.position.x(), c.position.y(), c.position.z());
		updateSteinerGridTargetBounds(p.position.x(), p.position.y(), p.position.z());
	}
}

void TriangulationManager::removeVisibilityPair(RayPath* r) {
	std::vector<int>& cvp = cameras_[r->cameraId].visiblePoints;
	cvp.erase(std::remove(cvp.begin(), cvp.end(), r->pointId), cvp.end());

	std::vector<int>& cnvp = cameras_[r->cameraId].newVisiblePoints;
	cnvp.erase(std::remove(cnvp.begin(), cnvp.end(), r->pointId), cnvp.end());

	std::vector<int>& pvc = points_[r->pointId].viewingCams;
	pvc.erase(std::remove(pvc.begin(), pvc.end(), r->cameraId), pvc.end());

	if (cvp.size() == 0) cout << "Orphan camera " << r->cameraId << endl; //TODO remove
	if (pvc.size() == 0) {
		pointsToBeRemovedId_.push_back(r->pointId);
	}
}

void TriangulationManager::removeRay(RayPath* r) {

	// Untrace the ray
	rayUntracing(r);

	// Remove the visibility pair
	removeVisibilityPair(r);

	// Remove it from the paths
	eraseRayPath(r);
}

RayPath* TriangulationManager::addRayPath(int cameraId, int pointId) {
	RayPath* r = new RayPath();

	r->cameraId = cameraId;
	r->pointId = pointId;

	const std::pair<int, int> k = std::pair<int, int>(cameraId, pointId);
	rayPaths_.insert(std::pair<const std::pair<int, int>, RayPath*>(k, r));

	camerasRayPaths_[cameraId].insert(r);
	pointsRayPaths_[pointId].insert(r);

	return r;
}
RayPath* TriangulationManager::getRayPath(int cameraId, int pointId) {
	if (!rayPaths_.count(pair<int, int>(cameraId, pointId))) {
		return addRayPath(cameraId, pointId);
	}

	const pair<int, int> k = pair<int, int>(cameraId, pointId);
	return rayPaths_.at(k);
}
void TriangulationManager::eraseRayPath(RayPath* r) {
	const std::pair<int, int> k = std::pair<int, int>(r->cameraId, r->pointId);
	rayPaths_.erase(pair<int, int>(k));

	camerasRayPaths_[r->cameraId].erase(r);
	pointsRayPaths_[r->pointId].erase(r);

	delete r;
}
std::set<RayPath*> TriangulationManager::getRayPathsFromCamera(int cameraId) {
	return camerasRayPaths_.at(cameraId);
}
std::set<RayPath*> TriangulationManager::getRayPathsFromPoint(int pointId) {
	return pointsRayPaths_.at(pointId);
}

void TriangulationManager::rayTracingFromAllCam() {
	Chronometer chronoEverything;

	if (raysToBeUntraced_.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		std::vector<pair<int, int>> raysToBeUntracedVector(raysToBeUntraced_.begin(), raysToBeUntraced_.end());

#pragma omp parallel for
		for (int i = 0; i < raysToBeUntracedVector.size(); i++) {
			pair<int, int> cIndex_pIndex = raysToBeUntracedVector[i];
			rayUntracing(getRayPath(cIndex_pIndex.first, cIndex_pIndex.second));
		}

		// without parallelism:
//		for (auto cIndex_pIndex : raysToBeUntraced_) {
//			rayUntracing(getRayPath(cIndex_pIndex.first, cIndex_pIndex.second));
//		}

		raysToBeUntraced_.clear();

		chronoEverything.stop();
		if (conf_.timeStatsOutput) cout << "rayUntracing\t\t" << chronoEverything.getSeconds() << endl << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.timeStatsOutput) cout << "rayUntracing\t\tSkipped" << endl << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

//#pragma omp parallel for
//	for (int n = 0; n < 10; ++n) {
//		bool a = 1;
//		for (int m = 0; m < 10000000; m++)
//			if (a) a = 0;
//			else a = 1;
//		if (!a) cout << a;
//		printf("\t%d", n);
//	}
//	printf(".\n");

	if (raysToBeTraced_.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		int tracedRays = raysToBeTraced_.size();

		for (auto cIndex_pIndex : raysToBeTraced_) {
			raysToBeRetraced_.erase(cIndex_pIndex);
			rayTracing(cIndex_pIndex.first, cIndex_pIndex.second);
		}

		raysToBeTraced_.clear();

		chronoEverything.stop();
		if (conf_.timeStatsOutput) cout << "rayTracing\t\t" << chronoEverything.getSeconds() << "\t/\t" << tracedRays << endl << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.timeStatsOutput) cout << "rayTracing\t\tSkipped" << endl << endl;
		timeStatsFile_ << 0.0 << ", ";
	}

	if (raysToBeRetraced_.size()) {
		chronoEverything.reset();
		chronoEverything.start();

		int retracedRays = raysToBeRetraced_.size();

		std::vector<pair<int, int>> raysToBeRetracedVector(raysToBeRetraced_.begin(), raysToBeRetraced_.end());

#pragma omp parallel for
		for (int i = 0; i < raysToBeRetracedVector.size(); i++) {
			pair<int, int> cIndex_pIndex = raysToBeRetracedVector[i];
//			cout << omp_get_thread_num() << " / " << omp_get_num_threads() << "\tr b " << cIndex_pIndex.first << ", " << cIndex_pIndex.second << endl;
			rayRetracing(cIndex_pIndex.first, cIndex_pIndex.second);
//			cout << omp_get_thread_num() << " / " << omp_get_num_threads() << "\tr e " << cIndex_pIndex.first << ", " << cIndex_pIndex.second << endl;
		}

		// without parallelism:
//		for (auto cIndex_pIndex : raysToBeRetraced_) {
//			rayRetracing(cIndex_pIndex.first, cIndex_pIndex.second);
//		}

		raysToBeRetraced_.clear();

		for (auto cell : newCells_) {
			if (dt_.is_cell(cell)) {
				cell->info().markOld();
			} else {
				; //cerr << "new cell was found dead :C" << endl;
			}
		}
		newCells_.clear();

		chronoEverything.stop();
		if (conf_.timeStatsOutput) cout << "rayRetracing\t\t" << chronoEverything.getSeconds() << "\t/\t" << retracedRays << endl << endl;
		timeStatsFile_ << chronoEverything.getSeconds() << ", ";
	} else {
		if (conf_.timeStatsOutput) cout << "rayRetracing\t\tSkipped" << endl << endl;
		timeStatsFile_ << 0.0 << ", ";
	}
}

void TriangulationManager::rayTracing(int cameraIndex, int pointIndex, bool retrace) {
	Vertex3D_handle vertexHandle;
	PointD3 source, target;

	std::vector<Delaunay3::Cell_handle> incidentCells;
	Delaunay3::Cell_handle previousCell, currentCell, targetCell;
	Delaunay3::Locate_type lt;
	int li, lj;

#	pragma omp critical
	{
		if (retrace) countRetracedRays_++;
		else countTracedRays_++;

		vertexHandle = points_[pointIndex].vertexHandle;
		source = points_[pointIndex].position;
		target = cameras_[cameraIndex].position;
	}

	Segment constraint = Segment(source, target);

	bool firstExitFacetFound = false;
	long int iterationCount = 0;

	RayPath* rayPath = getRayPath(cameraIndex, pointIndex);
	if (!retrace && rayPath->path.size()) cerr << "TriangulationManager::rayTracing: ray path not empty before rayTracing for ray " << cameraIndex << ", " << pointIndex << endl;
	rayPath->path.clear();

	if (vertexHandle == NULL) {
		cerr << "TriangulationManager::rayTracing: ignoring ray because vertex not in triangulation (vertexHandle == NULL); ray " << cameraIndex << ", " << pointIndex << endl;
		return;
	}

	if (points_[pointIndex].notTriangulated) {
		cerr << "TriangulationManager::rayTracing: ignoring ray because vertex not in triangulation (notTriangulated == true); ray " << cameraIndex << ", " << pointIndex << endl;
		return;
	}

	rt2_ChronoFirstCell_.start();

	// incidentCells contains the cells incident to the point's vertex
	dt_.incident_cells(vertexHandle, std::back_inserter(incidentCells));

	if (incidentCells.size() == 0) std::cerr << "TriangulationManager::rayTracing: no incident cells found for ray " << cameraIndex << ", " << pointIndex << std::endl;
	if (vertexHandle == NULL) std::cerr << "TriangulationManager::rayTracing: vertexHandle is NULL for ray " << cameraIndex << ", " << pointIndex << std::endl;

	// Locate the target cell
	targetCell = dt_.locate(target, lt, li, lj);

	if (lt != Delaunay3::Locate_type::CELL) {
		// (WORKAROUND) moving camera position because overlapping a vertex for ray " << cameraIndex << ", " << pointIndex << endl;
		target = PointD3(target.x() + 0.0001 * conf_.steinerGridStepLength, target.y() + 0.0001 * conf_.steinerGridStepLength, target.z() + 0.0001 * conf_.steinerGridStepLength);
		constraint = Segment(source, target);

		targetCell = dt_.locate(target, lt, li, lj);

		if (lt != Delaunay3::Locate_type::CELL) {
			cerr << "TriangulationManager::rayTracing: (WORKAROUND) moving camera position because overlapping a vertex; didn't work, aborting rayTracing for ray " << cameraIndex << ", " << pointIndex << endl;
			return;
		}
	}

	// If one of the incident cells also contains the camera, the ray ends in that cell. Mark it and return
	for (auto i : incidentCells) {
		if (i == targetCell) {
			markCell(i, cameraIndex, pointIndex, rayPath->path, retrace);
			if (!retrace && conf_.enableRayMistrust) markRemovalCandidateRays(vertexHandle, i, incidentCells);
			return;
		}
	}

	// Otherwise there has to be a neighbour cell that intersects with the ray and it is the neighbour opposite to the point's vertex
	for (auto i : incidentCells) {

		// facetIndex is the index of the facet opposite to the point's vertex in the incident cell i
		int facetIndex = i->index(vertexHandle);

		// If the facet (i, facetIndex) intersects with the ray, the corresponding neighbour is the next cell,
		// then mark the first cell and initialise the first two cells (previous and current)
		if (CGAL::do_intersect(dt_.triangle(i, facetIndex), constraint)) {
			firstExitFacetFound = true;

			markCell(i, cameraIndex, pointIndex, rayPath->path, retrace);
			if (!retrace && conf_.enableRayMistrust) markRemovalCandidateRays(vertexHandle, i, incidentCells); // TODO also if retracing?

			previousCell = i;
			currentCell = i->neighbor(facetIndex);

			break;
		}
	}
	rt2_ChronoFirstCell_.stop();

	if (!firstExitFacetFound) {
		std::cerr << "firstExitFacet NOT FOUND for ray " << cameraIndex << ", " << pointIndex << "\tincidentCells.size(): " << incidentCells.size() << std::endl;
		return;
	}

	rt2_ChronoCellTraversing_.start();

	// Follow the ray through the triangulation and mark the cells
	do {
		iterationCount++;
		markCell(currentCell, cameraIndex, pointIndex, rayPath->path, retrace);
	} while (nextCellOnRay(currentCell, previousCell, targetCell, constraint) && iterationCount < 10000); // TODO no magic

	if (iterationCount >= 5000) cerr << "TriangulationManager::rayTracing: max iterations exceeded for ray " << cameraIndex << ", " << pointIndex << endl;

	rt2_ChronoCellTraversing_.stop();

}

void TriangulationManager::rayRetracing(int cameraIndex, int pointIndex) {
	rayTracing(cameraIndex, pointIndex, true);
}

void TriangulationManager::rayUntracing(RayPath* rayPath) {
	int cameraIndex = rayPath->cameraId;
	int pointIndex = rayPath->pointId;

	countUntracedRays_++;

	// Remove all the dead cells from the path
	rayPath->path.erase(std::remove_if(rayPath->path.begin(), rayPath->path.end(), [&](Delaunay3::Cell_handle cell) {
		return !dt_.is_cell(cell);
	}), rayPath->path.end());

	// For all the cells in the ray's path, do the opposite of rayTracing
	for (auto c : rayPath->path) {
		unmarkCell(c, cameraIndex, pointIndex);
	}

	// Remove all cells from the path (rayTracing will add them back)
	rayPath->path.clear();

}

bool TriangulationManager::nextCellOnRay(
		Delaunay3::Cell_handle& currentCell, Delaunay3::Cell_handle& previousCell, const Delaunay3::Cell_handle& targetCell, const Segment& constraint) {

	if (currentCell == targetCell) return false;

	int cachedFacetIndex = currentCell->info().getRayTracingLastFacetIndex();

	if (cachedFacetIndex > -1) {
		Delaunay3::Cell_handle candidateNextCell = currentCell->neighbor(cachedFacetIndex);

		rt2_TriedCachedIndices++;

		// Note: dt_.triangle(currentCell, facetIndex) is the facet common to currentCell and candidateNextCell
		if (candidateNextCell != previousCell && CGAL::do_intersect(dt_.triangle(currentCell, cachedFacetIndex), constraint)) {

			previousCell = currentCell;
			currentCell = candidateNextCell;

			rt2_SuccessfulCachedIndices++;

			return true;
		}
	}

	// From the current tetrahedron, find the facets that intersects with the ray, excluding the entry facet
	for (int facetIndex = 0; facetIndex < 4; facetIndex++) {

		// Since the cached facet index has already been checked, don't try again
		if (cachedFacetIndex == facetIndex) continue;

		Delaunay3::Cell_handle candidateNextCell = currentCell->neighbor(facetIndex);

		// Note: dt_.triangle(currentCell, facetIndex) is the facet common to currentCell and candidateNextCell
		if (candidateNextCell != previousCell && CGAL::do_intersect(dt_.triangle(currentCell, facetIndex), constraint)) {

#pragma omp critical (cellUpdating)
			{
				currentCell->info().setRayTracingLastFacetIndex(facetIndex);
			}
			previousCell = currentCell;
			currentCell = candidateNextCell;

			return true;
		}
	}

	cerr << "TriangulationManager::nextCellOnRay: no next cell found" << endl;

	return false;
}

void TriangulationManager::markCell(Delaunay3::Cell_handle& c, const int cameraIndex, const int pointIndex, std::vector<Delaunay3::Cell_handle>& path, bool onlyMarkNewCells) {

#pragma omp critical (cellUpdating)
	{

		path.push_back(c);

		if (!onlyMarkNewCells || c->info().isNew()) {

			rt2_ChronoNeighboursD1WeightUpdate_.start();
			c->info().incrementNonConicFreeVote(1);
			c->info().incrementFreeVote(conf_.w_1);
			rt2_ChronoNeighboursD1WeightUpdate_.stop();

			rt2_ChronoNeighboursD2WeightUpdate_.start();
			c->info().addIntersection(cameraIndex, pointIndex);
			rt2_ChronoNeighboursD2WeightUpdate_.stop();

			rt2_ChronoNeighboursD1WeightUpdate_.start();
			for (int in1 = 0; in1 < 4; in1++) {
				Delaunay3::Cell_handle n1 = c->neighbor(in1);

				if (!onlyMarkNewCells || c->info().isNew()) {
					n1->info().incrementNonConicFreeVote(1);
					n1->info().incrementFreeVote(conf_.w_2);
				}

				for (int in2 = 0; in2 < 4; in2++) {
					Delaunay3::Cell_handle n2 = n1->neighbor(in2);

					if (!onlyMarkNewCells || c->info().isNew()) {
						n2->info().incrementNonConicFreeVote(1);
						n2->info().incrementFreeVote(conf_.w_3);
					}

				}
			}

			rt2_ChronoNeighboursD1WeightUpdate_.stop();

		}

	}
}

void TriangulationManager::unmarkCell(Delaunay3::Cell_handle& c, const int cameraIndex, const int pointIndex) {

#pragma omp critical (cellUpdating)
	{

		rt2_ChronoNeighboursD1WeightUpdate_.start();
		c->info().decrementNonConicFreeVote(1);
		c->info().decrementFreeVote(conf_.w_1);
		rt2_ChronoNeighboursD1WeightUpdate_.stop();

		rt2_ChronoNeighboursD2WeightUpdate_.start();
		c->info().removeIntersection(cameraIndex, pointIndex);
		rt2_ChronoNeighboursD2WeightUpdate_.stop();

		rt2_ChronoNeighboursD1WeightUpdate_.start();
		for (int in1 = 0; in1 < 4; in1++) {
			Delaunay3::Cell_handle n1 = c->neighbor(in1);

			n1->info().decrementNonConicFreeVote(1);
			n1->info().decrementFreeVote(conf_.w_2);

			for (int in2 = 0; in2 < 4; in2++) {
				Delaunay3::Cell_handle n2 = n1->neighbor(in2);

				n2->info().decrementNonConicFreeVote(1);
				n2->info().decrementFreeVote(conf_.w_3);
			}
		}
		rt2_ChronoNeighboursD1WeightUpdate_.stop();

	}
}

void TriangulationManager::markRemovalCandidateRays(Vertex3D_handle& v, Delaunay3::Cell_handle& c, std::vector<Delaunay3::Cell_handle>& incidentCells) {
	std::vector<Delaunay3::Cell_handle> neighbours;
	for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++)
		neighbours.push_back(c->neighbor(neighbourIndex));

	// Select cells i that are incident to vertex v and opposed to cell c (not neighbours),
	// and for each of them increment mistrust vote on intersecting rays and add them to raysCandidateToBeRemoved_.
	for (auto i : incidentCells) {
		if (i != c && i != neighbours[0] && i != neighbours[1] && i != neighbours[2] && i != neighbours[3]) {
			int intersectionsCount = i->info().getIntersections().size();
			for (auto intersection : i->info().getIntersections()) {
				getRayPath(intersection.first, intersection.second)->mistrustVote += conf_.w_m / intersectionsCount;
				raysCandidateToBeRemoved_.insert(pair<int, int>(intersection.first, intersection.second));
			}
		}
	}

	// TODO can take into account the freeVote of cell i (to avoid mass removing)?
}

void TriangulationManager::shrinkManifold(const std::set<index3>& enclosingVolumeMapIndices) {
	Chronometer chronoEverything;

	if (conf_.debugOutput) saveBoundary(0, 0);

	chronoEverything.reset();
	chronoEverything.start();

	manifoldManager_->shrinkManifold(enclosingVolumeMapIndices, currentEnclosingVersion_);

	chronoEverything.stop();
	if (conf_.timeStatsOutput) cout << "\tshrink\t\t" << chronoEverything.getSeconds() << endl << endl;
	timerShrinkTime_ += chronoEverything.getSeconds();

	if (conf_.debugOutput) saveBoundary(0, 1);

//	if (conf_.debugOutput) manifoldManager_->checkBoundaryIntegrity();

	chronoEverything.reset();
	chronoEverything.start();

	manifoldManager_->shrinkSeveralAtOnce(enclosingVolumeMapIndices, currentEnclosingVersion_);

	chronoEverything.stop();
	if (conf_.timeStatsOutput) cout << "\tshrinkSeveral\t" << chronoEverything.getSeconds() << endl << endl;
	timerShrinkSeveralTime_ += chronoEverything.getSeconds();

	if (conf_.debugOutput) saveBoundary(0, 2);

//	if (conf_.debugOutput) manifoldManager_->checkBoundaryIntegrity();

	chronoEverything.reset();
	chronoEverything.start();

	manifoldManager_->shrinkManifold(enclosingVolumeMapIndices, currentEnclosingVersion_);

	chronoEverything.stop();
	if (conf_.timeStatsOutput) cout << "\tshrink\t\t" << chronoEverything.getSeconds() << endl << endl;
	timerShrinkTime_ += chronoEverything.getSeconds();

	if (conf_.debugOutput) saveBoundary(0, 3);

//	if (conf_.debugOutput) manifoldManager_->checkBoundaryIntegrity();

	chronoEverything.reset();
	chronoEverything.start();

	manifoldManager_->shrinkSeveralAtOnce(enclosingVolumeMapIndices, currentEnclosingVersion_);

	if (conf_.timeStatsOutput) cout << "\tshrinkSeveral\t" << chronoEverything.getSeconds() << endl << endl;
	timerShrinkSeveralTime_ += chronoEverything.getSeconds();

	if (conf_.debugOutput) saveBoundary(0, 4);

//	if (conf_.debugOutput) manifoldManager_->checkBoundaryIntegrity();

	chronoEverything.reset();
	chronoEverything.start();

	manifoldManager_->shrinkManifold(enclosingVolumeMapIndices, currentEnclosingVersion_);

	chronoEverything.stop();
	if (conf_.timeStatsOutput) cout << "\tshrink\t\t" << chronoEverything.getSeconds() << endl << endl;
	timerShrinkTime_ += chronoEverything.getSeconds();

	if (conf_.debugOutput) saveBoundary(0, 5);

}

void TriangulationManager::growManifold(const std::set<index3>& enclosingVolumeMapIndices) {
	Chronometer chronoEverything;

	if (conf_.debugOutput) saveBoundary(1, 0);

	chronoEverything.reset();
	chronoEverything.start();
	if (manifoldManager_->getBoundarySize() == 0) {
		double max = 0.0;
		bool maxFound = false;
		Delaunay3::Cell_handle startingCell;
		cout << "TriangulationManager::growManifold: \t\t boundary is empty, growing from the cell with highest vote" << endl;
		// If the boundary is still empty, start growing from the cell with highest vote
		//TODO from every cell containing cameras; otherwise disconnected spaces wouldn't all be grown

		for (Delaunay3::Finite_cells_iterator itCell = dt_.finite_cells_begin(); itCell != dt_.finite_cells_end(); itCell++) {

			if (itCell->info().getFreeVote() >= max) {
				max = itCell->info().getFreeVote();
				startingCell = itCell;
				maxFound = true;
			}
		}

		if (!maxFound) {
			cout << "TriangulationManager::growManifold: \t\t triangulation is still empty. Can't grow" << endl;
			return;
		}

		manifoldManager_->initAndGrowManifold(startingCell, enclosingVolumeMapIndices);
	} else {
		manifoldManager_->growManifold(enclosingVolumeMapIndices);
	}

//	if (conf_.debugOutput) manifoldManager_->checkBoundaryIntegrity();

	chronoEverything.stop();
	if (conf_.timeStatsOutput) cout << "growManifold\t\t" << chronoEverything.getSeconds() << endl << endl;
	timeStatsFile_ << chronoEverything.getSeconds() << ", ";

	if (conf_.debugOutput) saveBoundary(1, 1);

//	if (conf_.debugOutput) manifoldManager_->checkBoundaryIntegrity();

	chronoEverything.reset();
	chronoEverything.start();
	manifoldManager_->growSeveralAtOnce(enclosingVolumeMapIndices);
	chronoEverything.stop();
	if (conf_.timeStatsOutput) cout << "growManifoldSev\t\t" << chronoEverything.getSeconds() << endl << endl;
	timeStatsFile_ << chronoEverything.getSeconds() << ", ";

	if (conf_.debugOutput) saveBoundary(1, 2);

//	if (conf_.debugOutput) manifoldManager_->checkBoundaryIntegrity();

	chronoEverything.reset();
	chronoEverything.start();
	manifoldManager_->growManifold(enclosingVolumeMapIndices);
	chronoEverything.stop();
	if (conf_.timeStatsOutput) cout << "growManifold\t\t" << chronoEverything.getSeconds() << endl << endl;
	timeStatsFile_ << chronoEverything.getSeconds() << ", ";

	if (conf_.debugOutput) saveBoundary(1, 3);
}

void TriangulationManager::saveManifold(const std::string filename) {
	manifoldManager_->getOutputManager()->writeMeshToOff(filename);

	if (conf_.debugOutput) {

		std::vector<Segment> rays;
		for (auto rayPath : rayPaths_) {

			PointD3 source = points_[rayPath.second->pointId].position;
			PointD3 target = cameras_[rayPath.second->cameraId].position;
			Segment constraint = Segment(source, target);

			rays.push_back(constraint);
		}
//		manifoldManager_->getOutputManager()->writeRaysToOFF("output/all_rays/rays",
//				std::vector<int> { iterationCounter_ }, rays);
		cout << "saving " << rays.size() << "rays" << endl;
		manifoldManager_->getOutputManager()->writeRaysToOFF(conf_.outputFolder, std::vector<int> { iterationCounter_ }, rays);
	}
}

void TriangulationManager::saveBoundary(int i, int j) {
	std::set<Delaunay3::Cell_handle> b = manifoldManager_->getBoundaryCells();
	if (b.size()) manifoldManager_->getOutputManager()->writeTetrahedraToOFF("output/boundary/boundary", std::vector<int> { iterationCounter_, i, j }, b);
}

bool TriangulationManager::integrityCheck() {
	return manifoldManager_->checkBoundaryIntegrity();
}

void TriangulationManager::initSteinerPointGridAndBound() {
	std::vector<pair<PointD3, Delaunay3DVertexInfo>> newPoints;

	for (float x = sgCurrentMinX_; x <= sgCurrentMaxX_; x += stepX_)
		for (float y = sgCurrentMinY_; y <= sgCurrentMaxY_; y += stepY_)
			for (float z = sgCurrentMinZ_; z <= sgCurrentMaxZ_; z += stepZ_)
				newPoints.push_back(pair<PointD3, Delaunay3DVertexInfo>(PointD3(x, y, z), Delaunay3DVertexInfo(nextSteinerPointId_--)));

	dt_.insert(newPoints.begin(), newPoints.end());

	countInsertedSteinerPoints_ += newPoints.size();

	if (conf_.timeStatsOutput) cout << "TriangulationManager::initSteinerPointGridAndBound:   \t\t added Steiner points: \t\t\t" << newPoints.size() << endl;
	if (conf_.timeStatsOutput) cout << "TriangulationManager::initSteinerPointGridAndBound:   \t\t total Steiner points: \t\t\t" << -nextSteinerPointId_ - 2 << endl;
	if (conf_.timeStatsOutput)
		cout << "TriangulationManager::initSteinerPointGridAndBound:   \t\t Steiner grid side lengths: \t\t" << "x: " << (int) (sgCurrentMaxX_ - sgCurrentMinX_ + 1.5) << "\ty: " << (int) (sgCurrentMaxY_ - sgCurrentMinY_ + 1.5) << "\tz: " << (int) (sgCurrentMaxZ_ - sgCurrentMinZ_ + 1.5) << endl;
}

void TriangulationManager::updateSteinerPointGridAndBound() {
	std::vector<pair<PointD3, Delaunay3DVertexInfo>> newPoints;

	// Prolong the grid on the positive x semi axis
	while (sgCurrentMaxX_ < sgMaxX_ + stepX_) {
		float x = sgCurrentMaxX_ + stepX_;
		for (float y = sgCurrentMinY_; y <= sgCurrentMaxY_; y += stepY_)
			for (float z = sgCurrentMinZ_; z <= sgCurrentMaxZ_; z += stepZ_)
				newPoints.push_back(pair<PointD3, Delaunay3DVertexInfo>(PointD3(x, y, z), Delaunay3DVertexInfo(nextSteinerPointId_--)));
		sgCurrentMaxX_ += stepX_;
	}

	// Prolong the grid on the negative x semi axis
	while (sgCurrentMinX_ > sgMinX_ - stepX_) {
		float x = sgCurrentMinX_ - stepX_;
		for (float y = sgCurrentMinY_; y <= sgCurrentMaxY_; y += stepY_)
			for (float z = sgCurrentMinZ_; z <= sgCurrentMaxZ_; z += stepZ_)
				newPoints.push_back(pair<PointD3, Delaunay3DVertexInfo>(PointD3(x, y, z), Delaunay3DVertexInfo(nextSteinerPointId_--)));
		sgCurrentMinX_ -= stepX_;
	}

	// Prolong the grid on the positive y semi axis
	while (sgCurrentMaxY_ < sgMaxY_ + stepY_) {
		float y = sgCurrentMaxY_ + stepY_;
		for (float x = sgCurrentMinX_; x <= sgCurrentMaxX_; x += stepX_)
			for (float z = sgCurrentMinZ_; z <= sgCurrentMaxZ_; z += stepZ_)
				newPoints.push_back(pair<PointD3, Delaunay3DVertexInfo>(PointD3(x, y, z), Delaunay3DVertexInfo(nextSteinerPointId_--)));
		sgCurrentMaxY_ += stepY_;
	}

	// Prolong the grid on the negative y semi axis
	while (sgCurrentMinY_ > sgMinY_ - stepY_) {
		float y = sgCurrentMinY_ - stepY_;
		for (float x = sgCurrentMinX_; x <= sgCurrentMaxX_; x += stepX_)
			for (float z = sgCurrentMinZ_; z <= sgCurrentMaxZ_; z += stepZ_)
				newPoints.push_back(pair<PointD3, Delaunay3DVertexInfo>(PointD3(x, y, z), Delaunay3DVertexInfo(nextSteinerPointId_--)));
		sgCurrentMinY_ -= stepY_;
	}

	// Prolong the grid on the positive z semi axis
	while (sgCurrentMaxZ_ < sgMaxZ_ + stepZ_) {
		float z = sgCurrentMaxZ_ + stepZ_;
		for (float x = sgCurrentMinX_; x <= sgCurrentMaxX_; x += stepX_)
			for (float y = sgCurrentMinY_; y <= sgCurrentMaxY_; y += stepY_)
				newPoints.push_back(pair<PointD3, Delaunay3DVertexInfo>(PointD3(x, y, z), Delaunay3DVertexInfo(nextSteinerPointId_--)));
		sgCurrentMaxZ_ += stepZ_;
	}

	// Prolong the grid on the negative z semi axis
	while (sgCurrentMinZ_ > sgMinZ_ - stepZ_) {
		float z = sgCurrentMinZ_ - stepZ_;
		for (float x = sgCurrentMinX_; x <= sgCurrentMaxX_; x += stepX_)
			for (float y = sgCurrentMinY_; y <= sgCurrentMaxY_; y += stepY_)
				newPoints.push_back(pair<PointD3, Delaunay3DVertexInfo>(PointD3(x, y, z), Delaunay3DVertexInfo(nextSteinerPointId_--)));
		sgCurrentMinZ_ -= stepZ_;
	}

	dt_.insert(newPoints.begin(), newPoints.end());

	countInsertedSteinerPoints_ += newPoints.size();

	if (conf_.timeStatsOutput) cout << "TriangulationManager::updateSteinerPointGridAndBound: \t\t added Steiner points: \t\t\t" << newPoints.size() << endl;
	if (conf_.timeStatsOutput) cout << "TriangulationManager::initSteinerPointGridAndBound:   \t\t total Steiner points: \t\t\t" << -nextSteinerPointId_ - 2 << endl;
	if (conf_.timeStatsOutput)
		cout << "TriangulationManager::updateSteinerPointGridAndBound: \t\t Steiner grid side lengths: \t\t" << "x: " << (int) (sgCurrentMaxX_ - sgCurrentMinX_ + 1.5) << "\ty: " << (int) (sgCurrentMaxY_ - sgCurrentMinY_ + 1.5) << "\tz: " << (int) (sgCurrentMaxZ_ - sgCurrentMinZ_ + 1.5) << endl;

}

void TriangulationManager::updateSteinerGridTargetBounds(float x, float y, float z) {
	if (sgMinX_ > x) sgMinX_ = x;
	if (sgMaxX_ < x) sgMaxX_ = x;

	if (sgMinY_ > y) sgMinY_ = y;
	if (sgMaxY_ < y) sgMaxY_ = y;

	if (sgMinZ_ > z) sgMinZ_ = z;
	if (sgMaxZ_ < z) sgMaxZ_ = z;
}

bool TriangulationManager::insertVertex(PointReconstruction& point) {

	// If the point is marked to be moved but wasn't inserted, use the new position
	if (point.notTriangulated && point.toBeMoved) {
		point.toBeMoved = false;
		point.position = point.newPosition;
	}

	// Locate the new point
	Delaunay3::Locate_type lt;
	int li, lj;
	Delaunay3::Cell_handle c = dt_.locate(point.position, lt, li, lj);

	if (!point.notTriangulated) {
		std::cerr << "TriangulationManager::insertVertex: point is already marked as triangulated!" << std::endl;
	}

	// If there is already a vertex in the new point's position, do not insert it
	if (lt == Delaunay3::VERTEX) {
		return false;
	}

	std::set<pair<int, int>> raysToBeRetraced;

	// Insert in removedCells the cells that conflict with the new point Q, and a facet on the boundary of this hole in f.
	// These cells will be erased by insert_in_hole
	std::vector<Delaunay3::Cell_handle> removedCells;
	Delaunay3::Facet f;
	dt_.find_conflicts(point.position, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(removedCells));

	for (auto cell : removedCells)
		if (cell->info().getBoundaryFlag()) {
			cerr << "TriangulationManager::insertVertex: destroying boundary cells" << std::endl;
			return false;
		}

	for (auto removedCell : removedCells) {
		for (auto intersection : removedCell->info().getIntersections())
			raysToBeRetraced.insert(pair<int, int>(intersection.first, intersection.second));
	}

	// Schedule a reyRetracing for all the rays that intersected the removed cells
	for (auto ray : raysToBeRetraced)
		raysToBeRetraced_.insert(pair<int, int>(ray.first, ray.second));

	// Creates a new vertex by starring a hole. Delete all the cells describing the hole vecConflictCells, creates a new vertex hndlQ, and for each facet on the boundary of the hole f, creates a new cell with hndlQ as vertex.
	Vertex3D_handle vertexHandle = dt_.insert_in_hole(point.position, removedCells.begin(), removedCells.end(), f.first, f.second);

	// Set of the cells that were created to fill the hole, used by rayRetracing to restore the rays
	std::vector<Delaunay3::Cell_handle> newCellsFromHole;
	dt_.incident_cells(vertexHandle, std::inserter(newCellsFromHole, newCellsFromHole.begin()));
	newCells_.insert(newCellsFromHole.begin(), newCellsFromHole.end());

	point.notTriangulated = false;
	point.vertexHandle = vertexHandle;
	vertexHandle->info().setPointId(point.idReconstruction);

	// Schedule rayTracing for all rays between the point and the cameras viewing it
	for (auto cameraIndex : point.viewingCams)
		raysToBeTraced_.insert(pair<int, int>(cameraIndex, point.idReconstruction));

	return true;

}

bool TriangulationManager::moveVertex(int pointIndex) {
	PointReconstruction& point = points_[pointIndex];

	if (!point.toBeMoved) {
		return false;
	}

	Delaunay3::Vertex_handle vertexHandle = point.vertexHandle;

	if ((point.notTriangulated && vertexHandle != NULL) || (!point.notTriangulated && vertexHandle == NULL)) {
		std::cerr << "TriangulationManager::moveVertex: point " << pointIndex << " new xnor (vertexHandle == NULL)" << std::endl;
	}

	// If the point isn't in the triangulation, do nothing
	if (vertexHandle == NULL) {
		return 0;
	}

	if (vertexHandle->point() != point.position) {
		std::cerr << "TriangulationManager::moveVertex: inconsistent position between vertex handle and point position for point " << pointIndex << std::endl;
	}

	PointD3 initialPosition = vertexHandle->point();
	PointD3 newPosition = point.newPosition;

	if (utilities::distanceEucl(initialPosition, newPosition) <= conf_.minDistancePointPositionUpdate) {
		return false;
	}

	point.position = point.newPosition;
	point.toBeMoved = false;

	// Set of rays <cameraIndex, pointIndex> intersecting the hole that needs to be retraced
	std::set<pair<int, int>> raysToBeRetraced;

	// Step 0
	// Undo rayTracing for all cells on the rayPaths concerning the point and schedule the rayTracing on those rays
	for (auto rayPath : getRayPathsFromPoint(pointIndex)) {
		//rayUntracing(rayPath);
		raysToBeUntraced_.insert(pair<int, int>(rayPath->cameraId, rayPath->pointId));

		// rayTracing will be computed again when possible
		raysToBeTraced_.insert(pair<int, int>(rayPath->cameraId, rayPath->pointId));

	}

	std::set<Delaunay3::Cell_handle> deadCells;

	// Step 1
	// The incident cells will be removed when the vertex is removed from the triangulation
	std::set<Delaunay3::Cell_handle> setIncidentCells;
	dt_.incident_cells(vertexHandle, std::inserter(setIncidentCells, setIncidentCells.begin()));
	deadCells.insert(setIncidentCells.begin(), setIncidentCells.end());

	// Schedule retracing for all rays that intersect the cells that will be removed
	for (auto itCell : setIncidentCells) {
		for (auto intersection : itCell->info().getIntersections())
			raysToBeRetraced.insert(pair<int, int>(intersection.first, intersection.second));
	}

	for (auto cell : deadCells)
		if (cell->info().getBoundaryFlag()) {
			cerr << "TriangulationManager::moveVertex: destroying boundary cells;  point " << pointIndex << std::endl;
			return false;
		}

	// Step 2
	// Remove the vertex from the triangulation
	dt_.remove(vertexHandle);
	point.notTriangulated = true;

	// Step 3
	// Locate the point
	Delaunay3::Locate_type lt;
	int li, lj;
	Delaunay3::Cell_handle c = dt_.locate(newPosition, lt, li, lj);
	if (lt == Delaunay3::VERTEX) {
		cerr << "Error in FreespaceDelaunayAlgorithm::moveVertex(): Attempted to move a vertex to an already existing vertex location" << endl;
		return false;
	}

	// Get the cells in conflict with the new vertex, and a facet on the boundary of this hole in f.
	// These cells will also be removed from the triangulation when the new vertex is inserted
	std::vector<Delaunay3::Cell_handle> vecConflictCells;
	Delaunay3::Facet f;
	dt_.find_conflicts(newPosition, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(vecConflictCells));
	deadCells.insert(vecConflictCells.begin(), vecConflictCells.end());

	// Schedule retracing for all rays that intersect the cells that will be removed (again)
	for (auto it : vecConflictCells) {
		for (auto intersection : it->info().getIntersections())
			raysToBeRetraced.insert(pair<int, int>(intersection.first, intersection.second));
	}

	for (auto cell : vecConflictCells)
		if (cell->info().getBoundaryFlag()) {
			cerr << "TriangulationManager::moveVertex: destroying boundary cells;  point " << pointIndex << std::endl;
			return false;
		}

	// Step 4
	// Fill the hole by inserting the new vertex
	vertexHandle = dt_.insert_in_hole(newPosition, vecConflictCells.begin(), vecConflictCells.end(), f.first, f.second);
	point.notTriangulated = false;
	point.vertexHandle = vertexHandle;
	vertexHandle->info().setPointId(point.idReconstruction);

	// Vector of the cells that were created to fill the hole
	std::vector<Delaunay3::Cell_handle> newCellsFromHole;
	dt_.incident_cells(vertexHandle, std::inserter(newCellsFromHole, newCellsFromHole.begin()));
	newCells_.insert(newCellsFromHole.begin(), newCellsFromHole.end());

	// Step 8
	// Schedule retracing all rays that intersected removed cells
	for (auto ray : raysToBeRetraced)
		raysToBeRetraced_.insert(pair<int, int>(ray.first, ray.second));

	return true;

}

void TriangulationManager::removeVertex(int pointIndex) {
	PointReconstruction& point = points_[pointIndex];
	Delaunay3::Vertex_handle vertexHandle = point.vertexHandle;

	if ((point.notTriangulated && vertexHandle != NULL) || (!point.notTriangulated && vertexHandle == NULL)) {
		std::cerr << "TriangulationManager::removeVertex: point " << pointIndex << " new xnor (vertexHandle == NULL)" << std::endl;
	}

	// If the point isn't in the triangulation, do nothing
	if (vertexHandle == NULL || point.notTriangulated) {
		std::cerr << "TriangulationManager::removeVertex: trying to remove a vertex not in triangulation; point " << pointIndex << std::endl;
		return;
	}

	if (vertexHandle->point() != point.position) {
		std::cerr << "TriangulationManager::removeVertex: inconsistent position between vertex handle and point position for point " << pointIndex << std::endl;
	}

	// Set of rays <cameraIndex, pointIndex> intersecting the hole that needs to be retraced
	std::set<pair<int, int>> raysToBeRetraced;
	std::set<Delaunay3::Cell_handle> deadCells;

	// Step 1
	// The incident cells will be removed when the vertex is removed from the triangulation
	std::set<Delaunay3::Cell_handle> setIncidentCells;
	dt_.incident_cells(vertexHandle, std::inserter(setIncidentCells, setIncidentCells.begin()));
	deadCells.insert(setIncidentCells.begin(), setIncidentCells.end());

	// Schedule retracing for all rays that intersect the cells that will be removed
	for (auto itCell : setIncidentCells)
		for (auto intersection : itCell->info().getIntersections())
			raysToBeRetraced.insert(pair<int, int>(intersection.first, intersection.second));

	for (auto cell : deadCells)
		if (cell->info().getBoundaryFlag()) {
			cerr << "TriangulationManager::removeVertex: destroying boundary cells; vertex " << pointIndex << std::endl;
			return;
		}

	// Step 2
	// Remove the vertex from the triangulation
	dt_.remove(vertexHandle);

	point.notTriangulated = true;
	point.vertexHandle = NULL;
}

bool TriangulationManager::moveCameraConstraints(int cameraIndex) {
	CamReconstruction& camera = cameras_[cameraIndex];
	PointD3 camPosition = camera.position;
	PointD3 newCamPosition = camera.newPosition;

	if (!camera.toBeMoved) return false;

	if (utilities::distanceEucl(camPosition, newCamPosition) <= conf_.minDistancePointPositionUpdate) { //TODO remove?
		return false;
	}
	camera.position = camera.newPosition;
	camera.toBeMoved = false;

	for (auto rayPath : getRayPathsFromCamera(cameraIndex)) {

		//rayUntracing(rayPath);
		raysToBeUntraced_.insert(pair<int, int>(rayPath->cameraId, rayPath->pointId));

		// Step 7: rayTracing will be computed again when possible
		raysToBeTraced_.insert(pair<int, int>(rayPath->cameraId, rayPath->pointId));

		// remove all cells from the path (rayTracing will add them back anyway)
		rayPath->path.clear();
	}
	return true;
}

