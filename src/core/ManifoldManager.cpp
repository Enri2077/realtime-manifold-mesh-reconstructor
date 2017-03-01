/*
 * ManifoldManager.cpp
 *
 *  Created on: 26/giu/2015
 *      Author: andrea
 */

#include <ManifoldManager.h>
#include <OutputManager.h>
#include <utilities.hpp>
#include <sstream>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <omp.h>

using std::cout;
using std::cerr;
using std::endl;

ManifoldManager::ManifoldManager(Delaunay3& dt, ManifoldReconstructionConfig& conf) :
		dt_(dt), conf_(conf) {

	out_ = new OutputManager(dt_, boundaryCellsSpatialMap_, conf);
}

ManifoldManager::~ManifoldManager() {
	boundaryCellsSpatialMap_.clear();
	delete out_;
}

bool ManifoldManager::isInEnclosingVolume(Delaunay3::Cell_handle& c, const std::set<index3>& enclosingVolumeMapIndices,
		const long& currentEnclosingVersion, Chronometer& chronoEnclosingCache, Chronometer& chronoEnclosingCheck) {
	bool inEnclosingVolume = false;
	countEnclosingVolumeCacheTotal_++;
	chronoEnclosingCache.start();
	if (c->info().getEnclosingVersion() == currentEnclosingVersion) {
		countEnclosingVolumeCacheHit_++;
		inEnclosingVolume = c->info().isInEnclosingVolume();
		chronoEnclosingCache.stop();
	} else {
		countEnclosingCells_++;
		chronoEnclosingCache.stop();

		chronoEnclosingCheck.start();
		for (int vertexId = 0; vertexId < 4; ++vertexId) {
			auto& p = c->vertex(vertexId)->point();

			int i = std::floor(p.x() / conf_.steinerGridStepLength);
			int j = std::floor(p.y() / conf_.steinerGridStepLength);
			int k = std::floor(p.z() / conf_.steinerGridStepLength);

			if (enclosingVolumeMapIndices.count(index3(i, j, k))) {
				inEnclosingVolume = true;
				break;
			}
		}
		chronoEnclosingCheck.stop();

		c->info().setInEnclosingVolume(inEnclosingVolume, currentEnclosingVersion);
	}
	return inEnclosingVolume;
}

void ManifoldManager::shrinkManifold(const std::set<index3>& enclosingVolumeMapIndices, long currentEnclosingVersion) {
	int countQueueInitCells = 0, countInEnclosingVolume = 0, countShrinked = 0, countTotal = 0, countIterations = 1;
	Chronometer chronoQueueInit, chronoQueueInserting, chronoQueuePopping, chronoInserting, chronoEnclosing,
			chronoEnclosingCache, chronoEnclosingCheck, chronoTesting, chronoShrinking;

	int countIsInMinecraftEnclosingVolume = 0;

	bool fixedPoint = false;
	std::vector<Delaunay3::Cell_handle> cellsNotShrinked;
	std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess> queue;

	chronoQueueInit.start();

	for (index3 mapIndex : enclosingVolumeMapIndices) {
		std::set<Delaunay3::Cell_handle>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
		queue.insert(localBoundaryCells.begin(), localBoundaryCells.end());
	}

	countQueueInitCells = queue.size();

	chronoQueueInit.stop();

	while (!fixedPoint) {

		fixedPoint = true;

		while (!queue.empty()) {
			countTotal++;

			chronoQueuePopping.start();

			Delaunay3::Cell_handle c = *queue.begin();
			queue.erase(queue.begin());

			chronoQueuePopping.stop();

			chronoEnclosing.start();
			bool inEnclosingVolume = isInEnclosingVolume(c, enclosingVolumeMapIndices, currentEnclosingVersion,
					chronoEnclosingCache, chronoEnclosingCheck);

			chronoEnclosing.stop();

			if (inEnclosingVolume) countInEnclosingVolume++;
			else continue;

			if (!c->info().getManifoldFlag() || !c->info().getBoundaryFlag()) continue;

			if (!c->info().getManifoldFlag()) cerr << "ManifoldManager::shrinkManifold:\t wrong shrink order (trying to shrink non manifold cell)\t iteration: " << countIterations << endl;
			if (!c->info().getBoundaryFlag()) cerr << "ManifoldManager::shrinkManifold:\t wrong shrink order (non boundary cell in queue)\t iteration: " << countIterations << endl;
			if (c->info().getManifoldFlag() && !c->info().getBoundaryFlag()) cerr << "ManifoldManager::shrinkManifold:\t inconsistent boundary and manifold (m and !b)\t iteration: " << countIterations << endl;
			if (!c->info().getManifoldFlag() && c->info().getBoundaryFlag()) cerr << "ManifoldManager::shrinkManifold:\t inconsistent boundary and manifold (!m and b)\t iteration: " << countIterations << endl;

			chronoTesting.start();
			chronoSingleTestOverall_.start();
			if (singleCellTest(c)) {
				chronoSingleTestOverall_.stop();
				chronoTesting.stop();

				if (!c->info().getManifoldFlag()) cerr << "ManifoldManager::shrinkManifold:\t wrong shrink order\t iteration: " << countIterations << endl;

				countShrinked++;

				chronoShrinking.start();
				std::vector<Delaunay3::Cell_handle> newBoundaryCells;
				subCellAndUpdateBoundary(c, newBoundaryCells);
				chronoShrinking.stop();

				for (auto neighbour : newBoundaryCells) {
					if (neighbour->info().getManifoldFlag()) {
						chronoQueueInserting.start();

						queue.insert(neighbour);

						chronoQueueInserting.stop();
						fixedPoint = false;

					} else cerr << "ManifoldManager::shrinkManifold:\t subCellAndUpdateBoundary return non manifold cell\t iteration: " << countIterations << endl;
				}

			} else {
				chronoSingleTestOverall_.stop();
				chronoTesting.stop();

				chronoInserting.start();
				cellsNotShrinked.push_back(c);
				chronoInserting.stop();

			}
		}

		if (queue.empty() && !fixedPoint) {
			countIterations++;
			queue.insert(cellsNotShrinked.begin(), cellsNotShrinked.end());
			cellsNotShrinked.clear();
		}
	}

	if (conf_.timeStatsOutput) {
		cout << "ManifoldManager::shrinkManifold:\t\t\t\t queue init:\t\t\t\t" << chronoQueueInit.getSeconds() << " s\t / \t" << countQueueInitCells << endl;
		cout << "ManifoldManager::shrinkManifold:\t\t\t\t queue inserting:\t\t\t" << chronoQueueInserting.getSeconds() << " s" << endl;
		cout << "ManifoldManager::shrinkManifold:\t\t\t\t queue popping:\t\t\t\t" << chronoQueuePopping.getSeconds() << " s" << endl;
		cout << "ManifoldManager::shrinkManifold:\t\t\t\t inserting:\t\t\t\t" << chronoInserting.getSeconds() << " s" << endl;
		cout << "ManifoldManager::shrinkManifold:\t\t\t\t enclosing cache:\t\t\t" << chronoEnclosingCache.getSeconds() << " s" << endl;
		cout << "ManifoldManager::shrinkManifold:\t\t\t\t enclosing check:\t\t\t" << chronoEnclosingCheck.getSeconds() << " s" << endl;
		cout << "ManifoldManager::shrinkManifold:\t\t\t\t test:\t\t\t\t\t" << chronoTesting.getSeconds() << " s\t / \t" << countInEnclosingVolume << endl;
		cout << "ManifoldManager::shrinkManifold:\t\t\t\t shrink:\t\t\t\t" << chronoShrinking.getSeconds() << " s\t / \t" << countShrinked << endl;
		cout << "ManifoldManager::shrinkManifold:\t\t\t\t number_of_finite_cells:\t\t" << dt_.number_of_finite_cells() << endl;
	}

	countShrinkedSingular_+=countShrinked;
	countSingleTestOverall_+=countInEnclosingVolume;
}

void ManifoldManager::shrinkSeveralAtOnce(const std::set<index3>& enclosingVolumeMapIndices,
		long currentEnclosingVersion) {
	chronoIsRegular_.reset();
	functionProfileCounter_isRegular_ = 0;
	Chronometer chronoQueueInit, chronoResourceInit, chronoAddAndCheckManifoldness;

	// boundaryVertices will contain all the vertices on the boundary between the manifold and non manifold cells
	std::set<Delaunay3::Vertex_handle> boundaryVertices;

	chronoQueueInit.start();

	for (index3 mapIndex : enclosingVolumeMapIndices) {
		for (auto boundaryCell : boundaryCellsSpatialMap_[mapIndex]) {

			if (!dt_.is_cell(boundaryCell)) {
				std::cerr << "ManifoldManager::shrinkSeveralAtOnce: \t\t dead cell found in boundary!" << endl;
				continue;
			}

			if (!boundaryCell->info().getManifoldFlag()) std::cout << "ManifoldManager::shrinkSeveralAtOnce: \t\t non manifold cell in boundary" << endl;

			// boundaryCell is a cell in the boundary
			for (int faceIndex = 0; faceIndex < 4; ++faceIndex) {
				Delaunay3::Cell_handle neighbour = boundaryCell->neighbor(faceIndex);

				if (!neighbour->info().getManifoldFlag()) {
					// Here neighbour is a non manifold cell.
					// faceIndex is the index (in boundaryCell) of the face between boundaryCell and neighbour.
					// Thus, faceIndex identifies the faces on the boundary

					for (int vertexIndex = 0; vertexIndex < 4; vertexIndex++) {
						// The vertices of boundaryCell with index that satisfy the condition vertexIndex != faceIndex
						// are the ones composing the boundary face
						if (vertexIndex != faceIndex) boundaryVertices.insert(boundaryCell->vertex(vertexIndex));
					}

				}
			}
		}
	}

	chronoQueueInit.stop();

	// The following commented code runs the shrink function in parallel, but the resources need to be allocated more efficently
//#pragma omp parallel
//	{
//		// Code inside this region runs in parallel.
//		printf("Hello!\n");
//	}

//#pragma omp for schedule(dynamic, 3)
// for(int n=0; n<10; ++n) printf(" %d", n);
// printf(".\n");

//	chronoResourceInit.start();
//
//	std::vector<std::pair<Delaunay3::Vertex_handle, std::set<Delaunay3::Cell_handle>>> verticesResources;
//
//	for (auto v : boundaryVertices) {
//		std::pair<Delaunay3::Vertex_handle, std::set<Delaunay3::Cell_handle>> v_r;
//		v_r.first = v;
//		std::vector<Delaunay3::Vertex_handle> adjacentVertices;
//		dt_.adjacent_vertices(v, std::back_inserter(adjacentVertices));
//		for (auto adjacentVertex : adjacentVertices)
//			dt_.incident_cells(adjacentVertex, std::inserter(v_r.second, v_r.second.begin()));
//
//		verticesResources.push_back(v_r);
//	}
//
//	chronoResourceInit.stop();
//
//	chronoAddAndCheckManifoldness.start();
//#pragma omp parallel
//	{
//		std::pair<Delaunay3::Vertex_handle, std::set<Delaunay3::Cell_handle>> locked_v_r;
//
//		while (nextLockableVertex(locked_v_r, verticesResources)) {
//			subSeveralAndCheckManifoldness(locked_v_r.first);
//			unlockCells(locked_v_r);
//		}
//	}

	// TODO check non boundary vertices instead? May be faster and more effective
	for (auto v : boundaryVertices) {
		subSeveralAndCheckManifoldness(v);
	}
	chronoAddAndCheckManifoldness.stop();

	if (conf_.timeStatsOutput) {
		cout << "ManifoldManager::shrinkSeveralAtOnce:\t\t\t\t queue init:\t\t\t\t" << chronoQueueInit.getSeconds() << " s" << endl;
		cout << "ManifoldManager::shrinkSeveralAtOnce:\t\t\t\t resource init:\t\t\t\t" << chronoResourceInit.getSeconds() << " s" << endl;
		cout << "ManifoldManager::shrinkSeveralAtOnce:\t\t\t\t addAndCheckManifoldness:\t\t" << chronoAddAndCheckManifoldness.getSeconds() << " s\t / \t" << boundaryVertices.size() << endl;
		cout << "ManifoldManager::shrinkSeveralAtOnce:\t\t\t\t isRegular:\t\t\t\t" << chronoIsRegular_.getSeconds() << " s\t / \t" << functionProfileCounter_isRegular_ << endl;
	}
}

//TODO efficient resource (cells) locking to parallelise boundary and manifold set update (the following is not an efficient implementation)

bool ManifoldManager::nextLockableVertex(std::pair<Delaunay3::Vertex_handle, std::set<Delaunay3::Cell_handle>>& vertex_lockedResources,
		std::vector<std::pair<Delaunay3::Vertex_handle, std::set<Delaunay3::Cell_handle>>>& verticesResources) {
	bool lockableVertexFound = false;

#pragma omp critical (cellLocking)
	{
//		cout << "T " << omp_get_thread_num() << " \t( " << omp_get_num_threads() << ")\tnextLockableVertex" << endl;

		auto v_r = verticesResources.begin();
		for (; v_r < verticesResources.end(); v_r++) {
			std::set<Delaunay3::Cell_handle>& cells = (*v_r).second;
			if (lockCells(cells)) {
				vertex_lockedResources = *v_r;
				lockableVertexFound = true;
				break; // Lockable vertex found
			}
		}

		if(lockableVertexFound) verticesResources.erase(v_r);
	}

	return lockableVertexFound;
}

bool ManifoldManager::lockCells(std::set<Delaunay3::Cell_handle>& lockingCells) {
	bool canLock;
//	cout << "T " << omp_get_thread_num() << " \t( " << omp_get_num_threads() << ")\tlockCells" << endl;
	canLock = true;
	for (auto c : lockingCells)
		if (c->info().isLocked()) {
			canLock = false;
			break;
		}

	if (canLock) for (auto c : lockingCells)
		c->info().lock();

	return canLock;
}

void ManifoldManager::unlockCells(std::pair<Delaunay3::Vertex_handle, std::set<Delaunay3::Cell_handle>>& vertex_lockedResources) {
//	cout << "T " << omp_get_thread_num() << " \t( " << omp_get_num_threads() << ")\tunlockCells" << endl;
	for (auto c : vertex_lockedResources.second)
		c->info().unlock();
}

void ManifoldManager::initAndGrowManifold(Delaunay3::Cell_handle& startingCell,
		const std::set<index3>& enclosingVolumeMapIndices) {
	addCellAndUpdateBoundary(startingCell);
	growManifold(enclosingVolumeMapIndices);
}

void ManifoldManager::growManifold(const std::set<index3>& enclosingVolumeMapIndices) {
	int countQueueInitCells, countBoundaryInitCells;
	int countGrowned = 0, countTotal = 0, countIterations = 0, maxIterations = 5;
	Chronometer chronoQueueInit, chronoQueueInserting, chronoQueuePopping, chronoInserting, chronoTesting,
			chronoGrowing;

	bool fixedPoint = false;

	std::set<Delaunay3::Cell_handle, sortTetByIntersectionAndDefaultLess> queue;
	std::vector<Delaunay3::Cell_handle> cellsNotCarved;

	chronoQueueInit.start();

	for (index3 mapIndex : enclosingVolumeMapIndices)
		for (auto boundaryCell : boundaryCellsSpatialMap_[mapIndex]) {

			if (!dt_.is_cell(boundaryCell)) {
				cerr << "ManifoldManager::growManifold: \t\t dead cell found in boundary!" << endl;
				continue;
			}

			if (!boundaryCell->info().getManifoldFlag()) cerr << "ManifoldManager::growManifold: \t\t non manifold cell in boundary" << endl;

			for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++) {
				Delaunay3::Cell_handle neighbour = boundaryCell->neighbor(neighbourIndex);

				if (!neighbour->info().getManifoldFlag() && isFreespace(neighbour)) queue.insert(neighbour);
			}
		}

	if (!queue.size()) {
		if (conf_.timeStatsOutput) cout << "ManifoldManager::growManifold:\t\t no local boundary cells found, searching in all boundary" << endl;

		for (auto i_localBoundaryCells : boundaryCellsSpatialMap_)
			for (auto boundaryCell : i_localBoundaryCells.second) {

				if (!dt_.is_cell(boundaryCell)) {
					cerr << "ManifoldManager::growManifold: \t\t dead cell found in boundary!" << endl;
					continue;
				}

				if (!boundaryCell->info().getManifoldFlag()) cerr << "ManifoldManager::growManifold: \t\t non manifold cell in boundary" << endl;

				for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++) {
					Delaunay3::Cell_handle neighbour = boundaryCell->neighbor(neighbourIndex);

					if (!neighbour->info().getManifoldFlag() && isFreespace(neighbour)) queue.insert(neighbour);
				}
			}

		if (conf_.timeStatsOutput && !queue.size()) {
			cout << "ManifoldManager::growManifold:\t\t no boundary cells found." << endl;
			return;
		}
	}

	chronoQueueInit.stop();

	countQueueInitCells = queue.size();

	while (!fixedPoint && countIterations < maxIterations) {
		fixedPoint = true;

		while (!queue.empty()) {
			countTotal++;

			chronoQueuePopping.start();

			Delaunay3::Cell_handle c = *queue.rbegin();
			queue.erase(*queue.rbegin());

			chronoQueuePopping.stop();

			if (c->info().getManifoldFlag() || c->info().getBoundaryFlag()) continue;

			if (c->info().getBoundaryFlag()) cerr << "ManifoldManager::growManifold:\t wrong grow order (boundary cell in queue)\t iteration: " << countIterations << endl;
			if (c->info().getManifoldFlag() && !c->info().getBoundaryFlag()) cerr << "ManifoldManager::growManifold:\t inconsistent boundary and manifold (m and !b)\t iteration: " << countIterations << endl;
			if (!c->info().getManifoldFlag() && c->info().getBoundaryFlag()) cerr << "ManifoldManager::growManifold:\t inconsistent boundary and manifold (!m and b)\t iteration: " << countIterations << endl;
			if (c->info().getManifoldFlag()) cerr << "ManifoldManager::growManifold:\t wrong grow order\t iteration: " << countIterations << endl;

			chronoTesting.start();
			chronoSingleTestOverall_.start();
			if (singleCellTest(c)) {
				chronoSingleTestOverall_.stop();
				chronoTesting.stop();

				countGrowned++;

				chronoGrowing.start();
				addCellAndUpdateBoundary(c);
				chronoGrowing.stop();

				for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++) {
					Delaunay3::Cell_handle neighbour = c->neighbor(neighbourIndex);

					if (!neighbour->info().getManifoldFlag() && isFreespace(neighbour)) {
						chronoQueueInserting.start();

						queue.insert(neighbour);
						fixedPoint = false;

						chronoQueueInserting.stop();

					}
				}

			} else {
				chronoSingleTestOverall_.stop();
				chronoTesting.stop();

				chronoInserting.start();

				cellsNotCarved.push_back(c);

				chronoInserting.stop();
			}
		}

		queue.insert(cellsNotCarved.begin(), cellsNotCarved.end());
		cellsNotCarved.clear();

		countIterations++;
	}

	queue.clear();

	if (conf_.timeStatsOutput) {
		cout << "ManifoldManager::growManifold:\t\t\t\t\t countIterations:\t\t\t\t\t" << countIterations << endl;
		cout << "ManifoldManager::growManifold:\t\t\t\t\t countBoundaryInitCells:\t\t\t\t" << countBoundaryInitCells << endl;
		cout << "ManifoldManager::growManifold:\t\t\t\t\t countQueueInitCells:\t\t\t\t\t" << countQueueInitCells << endl;
		cout << "ManifoldManager::growManifold:\t\t\t\t\t queue init:\t\t\t\t" << chronoQueueInit.getSeconds() << " s" << endl;
		cout << "ManifoldManager::growManifold:\t\t\t\t\t queue inserting:\t\t\t" << chronoQueueInserting.getSeconds() << " s" << endl;
		cout << "ManifoldManager::growManifold:\t\t\t\t\t queue popping:\t\t\t\t" << chronoQueuePopping.getSeconds() << " s" << endl;
		cout << "ManifoldManager::growManifold:\t\t\t\t\t inserting:\t\t\t\t" << chronoInserting.getSeconds() << " s" << endl;
		cout << "ManifoldManager::growManifold:\t\t\t\t\t test:\t\t\t\t\t" << chronoTesting.getSeconds() << " s\t / \t" << countTotal << endl;
		cout << "ManifoldManager::growManifold:\t\t\t\t\t grow:\t\t\t\t\t" << chronoGrowing.getSeconds() << " s\t / \t" << countGrowned << endl;
		cout << "ManifoldManager::growManifold:\t\t\t\t\t number_of_finite_cells:\t\t" << dt_.number_of_finite_cells() << endl;
	}

	countGrownSingular_+=countGrowned;
}

void ManifoldManager::growSeveralAtOnce(const std::set<index3>& enclosingVolumeMapIndices) {
	chronoIsRegular_.reset();
	functionProfileCounter_isRegular_ = 0;
	Chronometer chronoQueueInit, chronoRemoveAndCheckManifoldness;
	int countTotal = 0, countSuccess = 0;

	// boundaryVertices will contain all the vertices on the boundary between the manifold and non manifold cells
	std::set<Delaunay3::Vertex_handle> boundaryVertices;

	chronoQueueInit.start();

	for (index3 mapIndex : enclosingVolumeMapIndices) {

		for (auto boundaryCell : boundaryCellsSpatialMap_[mapIndex]) {

			if (!dt_.is_cell(boundaryCell)) {
				std::cerr << "ManifoldManager::growSeveralAtOnce: \t\t dead cell found in boundary!" << endl;
				continue;
			}

			if (!boundaryCell->info().getManifoldFlag()) std::cout << "ManifoldManager::growSeveralAtOnce: \t\t non manifold cell in boundary" << endl;

			// boundaryCell is a cell in the boundary
			for (int faceIndex = 0; faceIndex < 4; ++faceIndex) {
				Delaunay3::Cell_handle neighbour = boundaryCell->neighbor(faceIndex);

				if (!neighbour->info().getManifoldFlag() && isFreespace(neighbour)) {
					// Here neighbour is a non manifold cell and free space, so it could be grown.
					// faceIndex is the index (in boundaryCell) of the face between boundaryCell and neighbour.
					// Thus, faceIndex identifies the faces on the boundary

					for (int vertexIndex = 0; vertexIndex < 4; vertexIndex++) {
						// The vertices of boundaryCell with index that satisfy the condition vertexIndex != faceIndex
						// are the ones composing the boundary face
						if (vertexIndex != faceIndex) boundaryVertices.insert(boundaryCell->vertex(vertexIndex));
					}

				}
			}
		}
	}

	if (conf_.timeStatsOutput && !boundaryVertices.size()) {
		cout << "ManifoldManager::growSeveralAtOnce:\t\t\t no local boundary cells found." << endl;
		return;
	}

	chronoQueueInit.stop();

	chronoRemoveAndCheckManifoldness.start();

	// TODO check non boundary vertices instead? May be faster and more effective
	for (auto v : boundaryVertices) {
		countTotal++;

		bool success = addSeveralAndCheckManifoldness(v);

		if (success) countSuccess++;
	}

	chronoRemoveAndCheckManifoldness.stop();

	if (conf_.timeStatsOutput) {
		cout << "ManifoldManager::growSeveralAtOnce:\t\t\t\t queue init:\t\t\t\t" << chronoQueueInit.getSeconds() << " s" << endl;
		cout << "ManifoldManager::growSeveralAtOnce:\t\t\t\t remove and check manifoldness:\t\t" << chronoRemoveAndCheckManifoldness.getSeconds() << " s" << endl;
		cout << "ManifoldManager::growSeveralAtOnce:\t\t\t\t isRegular:\t\t\t\t" << chronoIsRegular_.getSeconds() << " s\t / \t" << functionProfileCounter_isRegular_ << endl;
		cout << "ManifoldManager::growSeveralAtOnce:\t\t\t\t successfully growned:\t\t\t" << countSuccess << "\t / \t" << countTotal << endl;
	}
}

bool ManifoldManager::addSeveralAndCheckManifoldness(Delaunay3::Vertex_handle v) {
	std::vector<Delaunay3::Cell_handle> incidentCells, modifiedCells;
	std::vector<Delaunay3::Vertex_handle> adjacentVertices;
	dt_.incident_cells(v, std::back_inserter(incidentCells));
	dt_.adjacent_vertices(v, std::back_inserter(adjacentVertices));

	bool testFailed = false;

	for (auto ic : incidentCells) {
		if (isFreespace(ic)) {
			if (!ic->info().getManifoldFlag()) {
				modifiedCells.push_back(ic);
				ic->info().setManifoldFlag(true);
			}
		}
	}

	for (auto iv : adjacentVertices) {
		if (!isRegularProfiled(iv)) {
			testFailed = true;
			break;
		}
	}

	if (!testFailed && !isRegularProfiled(v)) {
		testFailed = true;
	}

	if (testFailed) {

		for (auto ic : modifiedCells)
			ic->info().setManifoldFlag(false);

		return false;

	} else {

		countGrownSeveral_+=modifiedCells.size();

		for (auto ic : modifiedCells)
			ic->info().setManifoldFlag(false);

		for (auto ic : modifiedCells)
			addCellAndUpdateBoundary(ic);

	}

	return true;
}

bool ManifoldManager::subSeveralAndCheckManifoldness(Delaunay3::Vertex_handle v) {
	std::vector<Delaunay3::Cell_handle> incidentCells, modifiedCells;
	std::vector<Delaunay3::Vertex_handle> adjacentVertices;
	dt_.incident_cells(v, std::back_inserter(incidentCells));
	dt_.adjacent_vertices(v, std::back_inserter(adjacentVertices));

	for (auto ic : incidentCells) {
		if (ic->info().getManifoldFlag()) {
			modifiedCells.push_back(ic);
			ic->info().setManifoldFlag(false);
		}
	}

	bool testFailed = false;

	for (auto iv : adjacentVertices) {
		if (!isRegularProfiled(iv)) {
			testFailed = true;
			break;
		}
	}

	if (!testFailed && !isRegularProfiled(v)) {
		testFailed = true;
	}

	if (testFailed) {

		for (auto ic : modifiedCells)
			ic->info().setManifoldFlag(true);

		return false;

	} else {

		countShrinkedSeveral_+=modifiedCells.size();

		for (auto ic : modifiedCells)
			ic->info().setManifoldFlag(true);

		for (auto ic : modifiedCells) {
			std::vector<Delaunay3::Cell_handle> throwAway;
			subCellAndUpdateBoundary(ic, throwAway);
		}

	}

	return true;
}

void ManifoldManager::addCellAndUpdateBoundary(Delaunay3::Cell_handle& c) {

	// Add c to the manifold set and to the boundary if needed
	c->info().setManifoldFlag(true);

	std::vector<int> nonManifoldNeighbour;
	if (isBoundaryCell(c, nonManifoldNeighbour)) {
		insertInBoundary(c);
	} else {
		if (c->info().getBoundaryFlag()) {
			removeFromBoundary(c);
			cerr << "ManifoldManager::addCellAndUpdateBoundary: \t !isBoundaryCell and isBoundary() == true" << endl;
		}
	}

	// Check if the neighbour of the added cell still belongs to the boundary
	for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++) {
		Delaunay3::Cell_handle neighbour = c->neighbor(neighbourIndex);

		// If needed, remove the neighbour of c from the boundary

		// Note that isBoundary function returns the state flag of the cell, while
		// isBoundaryCell() checks for the current real state of the cell inside the triangulation after
		// the new cell has been added
		if (neighbour->info().getBoundaryFlag()) {

			if (!isBoundaryCell(neighbour)) {
				removeFromBoundary(neighbour);
			}

		} else {
			if (isBoundaryCell(neighbour)) {
				cerr << "ManifoldManager::addCellAndUpdateBoundary: \t isBoundaryCell == true and isBoundary() == false" << endl; // This should not happen, unless the flag is uncongruent maybe
				insertInBoundary(neighbour);
			}
		}
	}
}

/*
 * 	Remove the current cell from the manifold set and from the boundary set,
 * 	and add or remove its neighbours from the boundary set accordingly.
 */
void ManifoldManager::subCellAndUpdateBoundary(Delaunay3::Cell_handle& c,
		std::vector<Delaunay3::Cell_handle>& newBoundaryCells) {

	std::vector<int> notManifoldNeigh;
	isBoundaryCell(c, notManifoldNeigh);

	bool r = removeFromBoundary(c);

	if (!r) cout << "ManifoldManager::subCellAndUpdateBoundary: \t\t setting manifold flag to false but cell not removed from boundary" << endl;

	c->info().setManifoldFlag(false);

	// Check for each neighbour whether it should belong to the boundary set
	for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++) {
		Delaunay3::Cell_handle neighbour = c->neighbor(neighbourIndex);

		// If the neighbour wasn't in the boundary set before, it is now on the only condition of being in the manifold set,
		// in fact the remaining condition is satisfied. (Namely, it has at least a neighbour outside the manifold set, that is c).
		if (!neighbour->info().getBoundaryFlag()) {

			if (neighbour->info().getManifoldFlag()) {
				insertInBoundary(neighbour);
				newBoundaryCells.push_back(neighbour);
			}

		} else {
			if (!neighbour->info().getManifoldFlag()) {
				cerr << "ManifoldManager::subCellAndUpdateBoundary: \t\t cell such that iskeptManifold() == false && isBoundary() == true" << endl;
				removeFromBoundary(neighbour);
			}
		}
	}
}

bool ManifoldManager::isFreespace(Delaunay3::Cell_handle& c) {
	bool value;
	if (!conf_.enableInverseConic) {
		value = c->info().exceedsNonConicFreeVoteThreshold(conf_.nonConicFreeVoteThreshold);
	} else {
		value = c->info().exceedsFreeVoteThreshold(conf_.freeVoteThreshold);
	}
	return value;
}

/*
 *	If cellToBeAdded->info().isBoundary() is false, then insert cellToBeAdded in boundaryCells_ (maintaining the order) and set cellToBeAdded->info().isBoundary() to true.
 */
bool ManifoldManager::insertInBoundary(Delaunay3::Cell_handle& cellToBeAdded) {
	chronoInsertInBoundary_.start();

	if (!cellToBeAdded->info().getManifoldFlag()) cerr << "ManifoldManager::insertInBoundary: \t\t Violated precondition: Inserting a cell in boundary but iskeptManifold()==false" << endl;

	std::pair<std::set<Delaunay3::Cell_handle>::iterator, bool> i;
	std::set<index3> mapIndices;
	for (int vertexIndex = 0; vertexIndex < 4; vertexIndex++) {
		auto v = cellToBeAdded->vertex(vertexIndex);

		// The index of the grid's cube is the integer rounded down for each coordinate
		int i = std::floor(v->point().x() / conf_.steinerGridStepLength);
		int j = std::floor(v->point().y() / conf_.steinerGridStepLength);
		int k = std::floor(v->point().z() / conf_.steinerGridStepLength);
		mapIndices.insert(index3(i, j, k));
	}

	for (index3 mapIndex : mapIndices) {
		std::set<Delaunay3::Cell_handle>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
		i = localBoundaryCells.insert(cellToBeAdded);
	}

	if (cellToBeAdded->info().getBoundaryFlag() && !i.second) {
		cerr << "ManifoldManager::insertInBoundary: \t Uncongruent flag: isBoundary() == true but cell was NOT in boundary" << endl;
		return false;
	}

	cellToBeAdded->info().setBoundaryFlag(true);

	countInsertInBoundary_++;
	chronoInsertInBoundary_.stop();
	return true;

}

/*
 *	If cellToBeRemoved->info().isBoundary() is set to true, removes cellToBeRemoved from boundaryCells_ and set cellToBeRemoved->info().isBoundary() to false,
 *	otherwise does nothing.
 */
bool ManifoldManager::removeFromBoundary(Delaunay3::Cell_handle& cellToBeRemoved) {
	chronoRemoveFromBoundary_.start();

	int e = 0;

	std::set<index3> mapIndices;
	for (int vertexIndex = 0; vertexIndex < 4; vertexIndex++) {
		auto v = cellToBeRemoved->vertex(vertexIndex);

		// The index of the grid's cube is the integer rounded down for each coordinate
		int i = std::floor(v->point().x() / conf_.steinerGridStepLength);
		int j = std::floor(v->point().y() / conf_.steinerGridStepLength);
		int k = std::floor(v->point().z() / conf_.steinerGridStepLength);
		mapIndices.insert(index3(i, j, k));
	}

	for (index3 mapIndex : mapIndices) {
		std::set<Delaunay3::Cell_handle>& localBoundaryCells = boundaryCellsSpatialMap_[mapIndex];
		e += localBoundaryCells.erase(cellToBeRemoved);
	}

	if (!cellToBeRemoved->info().getBoundaryFlag() && e > 0) {
		cerr << "ManifoldManager::removeFromBoundary: \t Uncongruent flag: isBoundary() == false but cell was in boundary\t" << e << endl;
		return false;
	}

	cellToBeRemoved->info().setBoundaryFlag(false);

	countRemoveFromBoundary_++;
	chronoRemoveFromBoundary_.stop();
	return true;
}

// Returns true if c is a boundary cell. That is if c is in the manifold set and some of its neighbours aren't.
bool ManifoldManager::isBoundaryCell(Delaunay3::Cell_handle& c) {
	std::vector<int> toThrowAway;

	return isBoundaryCell(c, toThrowAway);
}

// Returns true if c is a boundary cell. That is if c is in the manifold set and some of its neighbours aren't.
// If c is a boundary cell, then neighboursNotManifold contains the indeces of the neighbours outside the boundary (that are not in the manifold set).
bool ManifoldManager::isBoundaryCell(Delaunay3::Cell_handle& c, std::vector<int>& neighboursNotManifold) {

	if (!c->info().getManifoldFlag()) {
		return false;
	} else {
		bool neighNotManifoldFound = false;

		for (int neighbourIndex = 0; neighbourIndex < 4; neighbourIndex++) {
			if (!c->neighbor(neighbourIndex)->info().getManifoldFlag()) {
				neighboursNotManifold.push_back(neighbourIndex);

				neighNotManifoldFound = true;
			}
		}
		return neighNotManifoldFound;
	}
}

bool ManifoldManager::isRegularProfiled(Delaunay3::Vertex_handle& v) {
	functionProfileCounter_isRegular_++;
	countIsRegularTest_++;
	counter_++;

	bool r = isRegular(v);

	if(r) functionProfileCounter_isRegularSuccessful_++;
	return r;
}

// TODO Merge first and second phase
bool ManifoldManager::isRegular(Delaunay3::Vertex_handle& v) {
	/*	Given |δV| as the manifold surface;
	 * 	The test is true if the triangles in δV including v can be ordered as t_0_ , ···, t_k−1_
	 * 	such that t_i_ ∩ t_(i+1)mod k_ is an edge, and such an edge is included in exactly two triangles t_i_ and t_j_
	 * 	In other words, the graph of the v-opposite edges in the δV triangles must form one and only one cycle.
	 *
	 * 	Based on this, three cases are possible:
	 * 	(a) There is no boundary surface passing through the vertex v because all incident cells are manifold,
	 * 	or (b) because no incident cell is manifold;
	 * 	(c)	There is a boundary surface passing through v because some incident cells are manifold and some others are not.
	 *
	 * 	In the first two cases the test is successful, in the third case it's necessary to test the following conditions on the graph:
	 * 	· (c_1) if the graph contains non binary vertices, implying the test is unsuccessful;
	 * 	· (c_2) if the graph contains multiple cycles, implying the test is unsuccessful;
	 * 	If none of these conditions are met, then the test is successful.
	 *
	 *
	 * 	Definitions:
	 * 	Binary Vertex (as in graph theory):
	 * 		A vertex that is part of exactly two edges.
	 * 	Binary Graph:
	 * 		A graph whose vertices are all binary vertices.
	 * 	v-opposite boundary edge (as in triangulation):
	 * 		An edge e such that the triangle t is composed of the vertices v, v1, v2. e is composed of v1 and v2.
	 * 		t ∈ δV, meaning that t is a face between two cells c1, c2 and c1 is in the manifold volume, c2 is not.
	 * 	v-opposite boundary vertices (as in triangulation):
	 * 		The vertices v1, v2 in the previous definition.
	 *
	 *	Note: only undirected graphs are considered.
	 */

	struct Edge {
		Delaunay3::Vertex_handle first, second;
		Edge(Delaunay3::Vertex_handle first, Delaunay3::Vertex_handle second) :
				first(first), second(second) {
		}
		Edge() {
			first = NULL;
			second = NULL;
		}
		inline bool operator<(const Edge& r) {
			Delaunay3::Vertex_handle x = std::min(first, second);
			Delaunay3::Vertex_handle y = std::min(r.first, r.second);
			if (x < y) return true;
			else if (y > x) return false;
			else return std::max(first, second) < std::max(r.first, r.second);
		}
		inline bool operator!=(const Edge& r) {
			return (first != r.first && first != r.second) || (second != r.second && second != r.first);
		}
		bool operator!() {
			return first == NULL; // second should be irrelevant, as either they both get assigned or neither
		}
		Delaunay3::Vertex_handle otherVertex(Delaunay3::Vertex_handle v1) {
			if (v1 != first) return first;
			else return second;
		}
	};

	// Can contain two edges like a pair, the contained edges are unique like in a set, the contained edges can be accessed as a vector.
	struct EdgePair {
		std::vector<Edge> edges;
		EdgePair() {
		}
		// Insert the Edge e.
		// Return false only when trying to insert a third edge different from the ones already present.
		// Returning false implies that a vertex is contained in three or more different edges
		// and means that a vertex is non binary.
		bool insert(Edge e) {
			if (edges.size() == 0) {
				edges.push_back(e);
				return true;
			} else if (edges.size() == 1) {
				if (e != edges[0]) {
					edges.push_back(e);
				}
				return true;
			} else {
				if (e != edges[0] && e != edges[1]) {
					edges.push_back(e);
					return false;
				}
				return true;
			}
		}
		// Precondition: The edges e1 and e2 are both and the only edges contained in this pair.
		// Given the edge e1, returns the other one.
		Edge otherEdge(Edge e1) {
			if (edges.size() != 2) cerr << "ManifoldManager::isRegular::Edge: \t EdgePair not quite a pair: " << edges.size() << " edge" << endl;
			if (e1 != edges[0]) return edges[0];
			else return edges[1];
		}
	};

	chronoIsRegular_.start();
	chronoIsRegularOverall_.start();

	std::vector<Delaunay3::Cell_handle> vIncidentCells;
	dt_.incident_cells(v, std::back_inserter(vIncidentCells));

	// Verify conditions a and b:
	bool allIncidentCellsAreManifold = true;
	bool noIncidentCellIsManifold = true;

	for (auto c : vIncidentCells) {
		if (allIncidentCellsAreManifold && !c->info().getManifoldFlag()) {
			allIncidentCellsAreManifold = false;
		}
		if (noIncidentCellIsManifold && c->info().getManifoldFlag()) {
			noIncidentCellIsManifold = false;
		}
		if (!allIncidentCellsAreManifold && !noIncidentCellIsManifold) break;
	}

	// If conditions a or b are met the test is successful, otherwise condition c is true.
	if (allIncidentCellsAreManifold || noIncidentCellIsManifold) {
		chronoIsRegularOverall_.stop();
		chronoIsRegular_.stop();
		return true;
	}

	/*	The previous code is equivalent to
	 *
	 *	for (auto c : vIncidentCells)
	 *		if (!c->info().getManifoldFlag()) {
	 *			allIncidentCellsAreManifold = false;
	 *			break;
	 *		}
	 *	if (allIncidentCellsAreManifold) return true;
	 *
	 *	for (auto c : vIncidentCells)
	 *		if (c->info().getManifoldFlag()) {
	 *			noIncidentCellIsManifold = false;
	 *			break;
	 *		}
	 *	if (noIncidentCellIsManifold) return true;
	 *
	 */

	// Here condition c is met.
	// To test conditions c_1 and c_2 it's necessary to populate the graph.
	std::unordered_set<Delaunay3::Vertex_handle> vOppositeBoundaryVertices;
	std::unordered_map<Delaunay3::Vertex_handle, EdgePair> vertexToEdgesMap;

	// firstEdge and firstVertex are any of the edges and vertices in the graph such that firstEdge contains firstVertex
	Edge firstEdge;
	Delaunay3::Vertex_handle firstVertex;
	bool foundNonBinaryVertices = false;

	// Find all v-opposite boundary edges and vertices
	for (auto incidentCell : vIncidentCells) {
		int vIndex = incidentCell->index(v);

		for (int i = 0; i < 4; i++)
			if (i != vIndex) {
				if (incidentCell->info().getManifoldFlag() != incidentCell->neighbor(i)->info().getManifoldFlag()) {
					// Here incidentCell is manifold and i is the index (in incidentCell) of a face
					// between incidentCell and another cell incident to v that is not manifold.
					// So the face identified by i is a boundary face containing v.

					int j = 0, m, n;
					// The edge given by (incidentCell, m, n) with m, n the two indices different from i and vIndex,
					// is on the boundary and opposite to the vertex v

					for (; j == i || j == vIndex; j++)
						;
					m = j; // m is equal to the first index different from i and vIndex
					for (j++; j == i || j == vIndex; j++)
						;
					n = j; // n is equal to the second (and last) index different from i and vIndex

					Edge e(incidentCell->vertex(m), incidentCell->vertex(n));

					// While inserting the edges in the map, also check condition c_1
					foundNonBinaryVertices |= !vertexToEdgesMap[incidentCell->vertex(m)].insert(e);
					foundNonBinaryVertices |= !vertexToEdgesMap[incidentCell->vertex(n)].insert(e);

					// If there are non binary vertices condition c_1 is met and the test is unsuccesful
					if (foundNonBinaryVertices) {
						chronoIsRegularOverall_.stop();
						chronoIsRegular_.stop();
						return false;
					}

					if (!firstEdge) {
						firstEdge = e;
						firstVertex = incidentCell->vertex(m);
					}

					vOppositeBoundaryVertices.insert(incidentCell->vertex(m));
					vOppositeBoundaryVertices.insert(incidentCell->vertex(n));

					// To debug: boundaryTriangles.push_back(dt_.triangle(incidentCell, i)); ... manifoldManager_->getOutputManager()->writeTrianglesToOFF("output/isRegular/v ", std::vector<int> { counter_ }, boundaryTriangles);
				}
			}
	}

	// Since condition c_1 is false, it's now guaranteed that the graph is binary, so all vertices must be part of a cycle.
	// Since conditions a and b weren't met, there has to be at least one cycle.
	// The test is succesful if there is exactly one cycle.
	// Given that all vertices are part of a cycle, to test whether there are multiple cycles (condition c_2),
	// it suffices to walk on a cycle and see if it contains all the vertices (the remaining vertices would be part of further cycles).

	if (!vOppositeBoundaryVertices.size()) {
		// <=> vOppositeBoundaryVertices.size() == 0 implies the graph is empty
		cerr << "ManifoldManager::isRegular \t\t NOT (allIncidentCellsAreManifold || noIncidentCellIsManifold) but graph is empty. Violated hypothesis: allIncidentCellsAreManifold || noIncidentCellIsManifold <=> empty graph" << endl;
		chronoIsRegularOverall_.stop();
		chronoIsRegular_.stop();
		return false;
	}
	if (allIncidentCellsAreManifold || noIncidentCellIsManifold) {
		cerr << "ManifoldManager::isRegular \t\t (allIncidentCellsAreManifold || noIncidentCellIsManifold) but graph is NOT empty. Violated hypothesis: allIncidentCellsAreManifold || noIncidentCellIsManifold <=> empty graph" << endl;
		chronoIsRegularOverall_.stop();
		chronoIsRegular_.stop();
		return false;
	}

	chronoIsRegularOverall_.stop();
	chronoIsRegularOverall2_.start();

	// Walk on the first cycle to count the number of its vertices and compare it to the total vertices
	Edge currentEdge = firstEdge;
	Delaunay3::Vertex_handle currentVertex = firstVertex;
	int verticesInThisCycleCount = 1;

	while (firstVertex != (currentVertex = currentEdge.otherVertex(currentVertex))) {

		// If the edges connected to the current vertex aren't exactly two
		// then this is not a cycle and the graph is not binary.
		// This condition should not be possible
		if (vertexToEdgesMap[currentVertex].edges.size() != 2) {
			cerr << "ManifoldManager::isRegular \t\t Violated hypothesis: Non binary graph" << endl;
			chronoIsRegularOverall2_.stop();
			chronoIsRegular_.stop();
			return false;
		}

		// Count the new vertex in the path
		verticesInThisCycleCount++;

		// Select the edge that is connected to the new vertex and is different from the previous one
		currentEdge = vertexToEdgesMap[currentVertex].otherEdge(currentEdge);
	}

	chronoIsRegularOverall2_.stop();
	chronoIsRegular_.stop();

	// Condition c_2 is met if there are more vertices than the ones in the walked cycle.
	// if c_2 is false the test is successful
	return verticesInThisCycleCount == vOppositeBoundaryVertices.size();

}


bool ManifoldManager::singleCellTest(Delaunay3::Cell_handle& cell) {
	bool isManifold = cell->info().getManifoldFlag();

	// Count the number of Facets in the intersection between cell and the current manifold
	int faceIndexI, faceIndexJ;
	int numF = 0;
	for (int faceIndex = 0; faceIndex < 4; ++faceIndex) {
		if (cell->neighbor(faceIndex)->info().getManifoldFlag() != isManifold) {
			numF++;
			if (numF == 1) faceIndexI = faceIndex;
			if (numF == 2) faceIndexJ = faceIndex;
		}
	}

	// If numF == 0, then the test is true only if both numV and numE are 0.
	// If either numV or numE are greater than zero, the test is already determined and false.
	if (numF == 0) {

		// If even one vertex satisfies the condition, the test is false (numV > 0)
		for (int curVertexId = 0; curVertexId < 4; ++curVertexId) {
			std::vector<Delaunay3::Cell_handle> incidentCells;
			dt_.incident_cells(cell->vertex(curVertexId), std::back_inserter(incidentCells));

			for (auto c : incidentCells)
				if (c->info().getManifoldFlag() != isManifold) return false; // shortcut

		}

		// If even one edge satisfies the condition, the test is false (numE > 0)
		for (int curEdgeId1 = 0; curEdgeId1 < 4; ++curEdgeId1) {
			for (int curEdgeId2 = curEdgeId1 + 1; curEdgeId2 < 4; ++curEdgeId2) {
				Delaunay3::Edge curEdge(cell, curEdgeId1, curEdgeId2);

				Delaunay3::Cell_circulator cellCirc = dt_.incident_cells(curEdge, cell);
				Delaunay3::Cell_circulator cellCircInit = dt_.incident_cells(curEdge, cell);

				do {
					if (cellCirc->info().getManifoldFlag() != isManifold) return false; // shortcut
					cellCirc++;
				} while (cellCirc != cellCircInit);

			}
		}

		return true;

	} else if (numF == 1) {

		// If numF == 1, then the test is true only if both numV and numE are 3,
		// but given that the condition is true for one and only one face (called face i),
		// then the vertex opposite to the face i (that is, vertex i), determines if the test is true.
		// In fact, the remaining vertices are incident to the cell adjacent to the face that already satisfied the condition,
		// hence the condition on those vertices is already satisfied.

		std::vector<Delaunay3::Cell_handle> incidentCells;
		dt_.incident_cells(cell->vertex(faceIndexI), std::back_inserter(incidentCells));

		for (auto c : incidentCells) {
			if (c->info().getManifoldFlag() != isManifold) {
				return false;
			}
		}

		// If the condition is true for one and the only one face i,
		// then the condition must be true for all edges composing the face i and whether or not the condition is true for the vertex opposite to the face i (that is, vertex i),
		// it is redundant to check for the remaining edges, since the cells incident to the vertex i are also incident to them.
		// condition false for vertex i implies condition false for remaining edges (i, -)
		// condition true for vertex i implies the test is already determined (and false).

		return true;

	} else if (numF == 2) {

		// If two of the faces are adjacent the cells that make the condition true, then the condition is implied for all vertices and for all but one edge.
		// The only edge for which the condition needs to be tested is the one connecting the vertices opposite to the faces satisfing the condition.
		// Calling such faces i and j, said edge is the edge (i, j).

		Delaunay3::Edge edgeIJ(cell, faceIndexI, faceIndexJ);
		Delaunay3::Cell_circulator cellCirc = dt_.incident_cells(edgeIJ, cell);
		Delaunay3::Cell_circulator cellCircInit = dt_.incident_cells(edgeIJ, cell);

		do {
			if (cellCirc->info().getManifoldFlag() != isManifold) {
				return false;
			}
			cellCirc++;
		} while (cellCirc != cellCircInit);

		return true;

	} else if (numF >= 3) {

		// If all or even just three faces satisfy the condition, then it is implied for all vertices and all edges to satisfy the condition.

		return true;

	}

}

bool ManifoldManager::checkBoundaryIntegrity() {
	std::cout << "ManifoldManager::checkBoundaryIntegrity: checking..." << endl;

	bool result = true;
	int count = 0;

	for (auto i_localBoundaryCells : boundaryCellsSpatialMap_) {
		std::set<Delaunay3::Cell_handle>& localBoundaryCells = i_localBoundaryCells.second;

		for (auto boundaryCell : localBoundaryCells) {
			count++;

			if (!dt_.is_cell(boundaryCell)) {
				cerr << "\t\t dead cell" << endl;
				result = false;
				continue;
			}

			bool manifoldTest = true;
			for (int vertexIndex = 0; vertexIndex < 4; vertexIndex++) {
				Delaunay3::Vertex_handle v = boundaryCell->vertex(vertexIndex);
				if (!isRegular(v)) manifoldTest = false;
			}

			if (!manifoldTest || !isFreespace(boundaryCell) || !isBoundaryCell(boundaryCell) || !boundaryCell->info().getBoundaryFlag()) {
				result = false;
				cerr << "\t\t tests failed on cell:" << endl;

				if (!manifoldTest) cerr << "\t\t\t\t\t has vertex v st !isRegular(v)" << endl;
				if (!isFreespace(boundaryCell)) cerr << "\t\t\t\t\t !isFreespace(boundaryCell)" << endl;
				if (!isBoundaryCell(boundaryCell)) cerr << "\t\t\t\t\t !isBoundaryCell(boundaryCell)" << endl;
				if (!boundaryCell->info().getManifoldFlag()) cerr << "\t\t\t\t\t !boundaryCell->info().getManifoldFlag()" << endl;
				if (!boundaryCell->info().getBoundaryFlag()) cerr << "\t\t\t\t\t !boundaryCell->info().getBoundaryFlag()" << endl;

				cerr << "\t\t\t info on cell:" << endl;

				cerr << "\t\t\t\t\t points:";
				for (int i = 0; i < 4; i++)
					cerr << "\t" << boundaryCell->vertex(i)->info().getPointId();
				cerr << endl;
			}

		}
	}

	cout << "#                checked " << count << ((count - 1) ? " cells" : " cell") << endl;

	if (!result) cout << "#########      Integrity Check FAILED!!!      #########" << endl << endl;
	else cout << "#                Integrity Check OK                   #" << endl << endl;

//	if (!result) throw new std::exception();
	return result;

}
