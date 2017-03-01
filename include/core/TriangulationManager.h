/*
 * TriangulationManager.h
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */


#ifndef TRIANGULATIONMANAGER_H_
#define TRIANGULATIONMANAGER_H_

#include <types_reconstructor.hpp>
#include <types_config.hpp>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>
#include <ManifoldManager.h>
#include <fstream>
#include <iostream>
#include <Ray.hpp>
#include <Chronometer.h>

/**
 * This class provides the API to manage the 2-manifold creation as explained in the
 * paper:
 *
 * Andrea Romanoni, Matteo Matteucci. Incremental Urban Manifold Reconstruction from a Sparse Edge-Points Cloud.
 * IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) 2015.
 *
 * The class builds a 3D Delaunay Triangulation, and keeps the information of the visibility updated incrementally.
 * In the same time it provides the functions to incrementally estimate a manifold mesh out of the triangulation.
 *
 * The moving point related functions (as explained in the paper: A. Romanoni, M. Matteucci.
 * Efficient moving point handling for incremental 3D manifold reconstruction.
 * International Conference on Image Analysis and Processing (ICIAP) 2015.) have not been tested in this library yet
 *
 *
 *
 */
class TriangulationManager {
public:
	TriangulationManager(ManifoldReconstructionConfig& conf);
	virtual ~TriangulationManager();

	void updateTriangulation();

	void addPoint(float x, float y, float z);
	void addPoint(float x, float y, float z, float r, float g, float b, float a);
	void movePoint(int id, float x, float y, float z);
	void movePoint(int id, float x, float y, float z, float r, float g, float b, float a);

	void addCamera(float x, float y, float z);
	void moveCamera(int id, float x, float y, float z);

	void addVisibilityPair(int cameraId, int pointId);

	void saveManifold(const std::string filename);
	void saveBoundary(int i, int j);

	bool integrityCheck();

	OutputManager* getOutputManager(){
		return manifoldManager_->getOutputManager();
	}

	std::ofstream timeStatsFile_, countStatsFile_;

private:

	RayPath* addRayPath(int cameraId, int pointId);
	RayPath* getRayPath(int cameraId, int pointId);
	void eraseRayPath(RayPath* r);
	std::set<RayPath*> getRayPathsFromCamera(int cameraId);
	std::set<RayPath*> getRayPathsFromPoint(int pointId);

	void rayTracingFromAllCam();
	void rayTracing(int cameraIndex, int pointIndex, bool retrace = false);
	void rayRetracing(int cameraIndex, int pointIndex);
	void rayUntracing(RayPath* path);

	bool nextCellOnRay(
			Delaunay3::Cell_handle& currentCell, Delaunay3::Cell_handle& previousCell, const Delaunay3::Cell_handle& targetCell,
			const Segment& constraint);
	void markCell(Delaunay3::Cell_handle& c, const int cameraIndex, const int pointIndex, std::vector<Delaunay3::Cell_handle>& path, bool onlyMarkNewCells);
	void unmarkCell(Delaunay3::Cell_handle& c, const int cameraIndex, const int pointIndex);

	void markRemovalCandidateRays(Vertex3D_handle& v, Delaunay3::Cell_handle& c, std::vector<Delaunay3::Cell_handle>& incidentCells);
	void removeRay(RayPath* r);
	void removeVisibilityPair(RayPath* r);

	void shrinkManifold(const std::set<index3>& enclosingVolumeMapIndices);
	void growManifold(const std::set<index3>& enclosingVolumeMapIndices);

	void initSteinerPointGridAndBound();
	void updateSteinerPointGridAndBound();
	void updateSteinerGridTargetBounds(float x, float y, float z);

	bool insertVertex(PointReconstruction& point);
	bool moveVertex(int pointIndex);
	void removeVertex(int pointIndex);
	bool moveCameraConstraints(int cameraIndex);


	Delaunay3 dt_;

	std::vector<CamReconstruction> cameras_;
	std::vector<PointReconstruction> points_;

	// Steiner vertices are not related to actual points, so their indices are negative numbers.
	// When not initialised the index is -1, further indices are the following negative integers.
	// The first assigned index is -2
	long int nextSteinerPointId_ = -2;

	std::map<std::pair<int, int>, RayPath*> rayPaths_;
	std::map<int, std::set<RayPath*>> camerasRayPaths_;
	std::map<int, std::set<RayPath*>> pointsRayPaths_;

	std::set<int> updatedCamerasId_;
	std::vector<int> movedPointsId_;
	std::vector<int> movedCamerasId_;

	std::vector<int> pointsToBeRemovedId_;
	std::vector<Delaunay3::Vertex_handle> verticesToBeRemoved_;

	std::set<Delaunay3::Cell_handle> newCells_;

	// raysToBeTraced_ contains all the rays <cameraIndex, pointIndex> that need to be traced by rayTracing.
	// See rayTracing
	std::set<std::pair<int, int>> raysToBeTraced_;

	// raysToBeRetraced_ contains all the rays <cameraIndex, pointIndex> that need to be retraced by rayRetracing.
	// A ray is retraced by rayRetracing if it was intersecting some cells that were removed from the triangulation.
	// See rayRetracing
	std::set<std::pair<int, int>> raysToBeRetraced_;

	std::set<std::pair<int, int>> raysToBeUntraced_;

	std::set<std::pair<int, int>> raysCandidateToBeRemoved_;

	// Steiner grid step on each axis
	float stepX_, stepY_, stepZ_;
	// Target Steiner grid bounds for each semi axis
	float sgMinX_, sgMaxX_, sgMinY_, sgMaxY_, sgMinZ_, sgMaxZ_;
	// Current Steiner grid bounds for each semi axis
	float sgCurrentMinX_, sgCurrentMaxX_, sgCurrentMinY_, sgCurrentMaxY_, sgCurrentMinZ_, sgCurrentMaxZ_;

	int iterationCounter_ = 0;
	long currentEnclosingVersion_ = 0;

	float timerShrinkTime_ = 0.0;
	float timerShrinkSeveralTime_ = 0.0;
	long rt2_CountNeighboursD1WeightUpdate_ = 0, rt2_CountNeighboursD2WeightUpdate_ = 0, rt2_SuccessfulCachedIndices = 0, rt2_TriedCachedIndices = 0, countTracedRays_ = 0, countRetracedRays_ = 0, countUntracedRays_ = 0, countInsertedSteinerPoints_ = 0;
	Chronometer rt2_ChronoUseless_, rt2_ChronoFirstCell_, rt2_ChronoCellTraversing_;
	Chronometer rt2_ChronoNeighboursD1Selection_, rt2_ChronoNeighboursD2Selection_, rt2_ChronoNeighboursD1WeightUpdate_, rt2_ChronoNeighboursD2WeightUpdate_;
	std::vector<Segment> movedPointsSegments_;

	ManifoldReconstructionConfig& conf_;
	ManifoldManager* manifoldManager_;
	std::ofstream fileOut_;
};

#endif /* TRIANGULATIONMANAGER_H_ */
