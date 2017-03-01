#ifndef MANIFOLDMANAGER_H_
#define MANIFOLDMANAGER_H_

#include <types_reconstructor.hpp>
#include <types_config.hpp>
#include <OutputManager.h>
#include <Chronometer.h>
#include <iostream>

class ManifoldManager {
public:
	ManifoldManager(Delaunay3& dt, ManifoldReconstructionConfig& conf);
	virtual ~ManifoldManager();

	size_t getBoundarySize() {
		long long int bSize = 0;
		for (auto i_lbc : boundaryCellsSpatialMap_) {
			bSize += i_lbc.second.size();
		}

		return bSize;
	}

	void shrinkManifold(const std::set<index3>& enclosingVolumeMapIndices, const long currentEnclosingVersion);

	void shrinkSeveralAtOnce(const std::set<index3>& enclosingVolumeMapIndices, long currentEnclosingVersion);

	void initAndGrowManifold(Delaunay3::Cell_handle& startingCell, const std::set<index3>& enclosingVolumeMapIndices);

	void growManifold(const std::set<index3>& enclosingVolumeMapIndices);

	void growSeveralAtOnce(const std::set<index3>& enclosingVolumeMapIndices);

	bool checkBoundaryIntegrity();

	const std::set<Delaunay3::Cell_handle> getBoundaryCells() const {
		std::set<Delaunay3::Cell_handle> boundaryCells;

		for (auto i_lbc : boundaryCellsSpatialMap_) {
			boundaryCells.insert(i_lbc.second.begin(), i_lbc.second.end());
		}

		return boundaryCells;
	}

	OutputManager* getOutputManager() {
		return out_;
	}

	Chronometer chronoInsertInBoundary_, chronoRemoveFromBoundary_, chronoIsRegularOverall_, chronoIsRegularOverall2_,
			chronoSingleTestOverall_;

	int countInsertInBoundary_ = 0, countRemoveFromBoundary_ = 0,
	countSingleTestOverall_ = 0, countIsRegularTest_ = 0, countEnclosingCells_ = 0, countEnclosingVolumeCacheHit_ = 0, countEnclosingVolumeCacheTotal_ = 0,
	countShrinkedSingular_ = 0, countShrinkedSeveral_ = 0, countGrownSingular_ = 0, countGrownSeveral_ = 0;

	Chronometer chronoIsRegular_;
	long functionProfileCounter_isRegular_ = 0, functionProfileCounter_isRegularSuccessful_ = 0;

private:

	/******************************************************/
	/**************Manifold check functions****************/
	/******************************************************/
	bool singleCellTest(Delaunay3::Cell_handle& i);
	bool addSeveralAndCheckManifoldness(Delaunay3::Vertex_handle v);
	bool subSeveralAndCheckManifoldness(Delaunay3::Vertex_handle v);
	bool isRegular(Delaunay3::Vertex_handle& v);
	bool isRegularProfiled(Delaunay3::Vertex_handle& v);

	bool isBoundaryCell(Delaunay3::Cell_handle& c);
	bool isBoundaryCell(Delaunay3::Cell_handle& c, std::vector<int>& neighNotManifold);
	bool isFreespace(Delaunay3::Cell_handle& c);

	bool isInEnclosingVolume(Delaunay3::Cell_handle& c, const std::set<index3>& enclosingVolumeMapIndices,
			const long& currentEnclosingVersion, Chronometer& chronoEnclosingCache, Chronometer& chronoEnclosingCheck);

	/******************************************************/
	/************Boundary update functions*****************/
	/******************************************************/
	void addCellAndUpdateBoundary(Delaunay3::Cell_handle& cell);
	void subCellAndUpdateBoundary(Delaunay3::Cell_handle& currentTet,
			std::vector<Delaunay3::Cell_handle>& newBoundaryTets);

	bool insertInBoundary(Delaunay3::Cell_handle& cellToTest);
	bool removeFromBoundary(Delaunay3::Cell_handle& cellToBeRemoved);

//	bool nextLockableVertex(Delaunay3::Vertex_handle& v, const std::set<Delaunay3::Vertex_handle>& boundaryVertices);
	bool nextLockableVertex(std::pair<Delaunay3::Vertex_handle, std::set<Delaunay3::Cell_handle>>& vertex_lockedResources,
			std::vector<std::pair<Delaunay3::Vertex_handle, std::set<Delaunay3::Cell_handle>>>& verticesResources);
	bool lockCells(std::set<Delaunay3::Cell_handle>& lockingCells);
	void unlockCells(std::pair<Delaunay3::Vertex_handle, std::set<Delaunay3::Cell_handle>>& vertex_lockedResources);

	Delaunay3& dt_;
	std::map<index3, std::set<Delaunay3::Cell_handle>> boundaryCellsSpatialMap_;

	ManifoldReconstructionConfig& conf_;
	OutputManager* out_ = NULL;

	int counter_ = 0;
};

#endif /* MANIFOLDMANAGER_H_ */
