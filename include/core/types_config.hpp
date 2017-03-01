//  Copyright 2014 Andrea Romanoni
//
//  This file is part of edgePointSpaceCarver.
//
//  edgePointSpaceCarver is free software: you can redistribute it
//  and/or modify it under the terms of the GNU General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version see
//  <http://www.gnu.org/licenses/>..
//
//  edgePointSpaceCarver is distributed in the hope that it will be
//  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

#ifndef TYPES_TEST_HPP_
#define TYPES_TEST_HPP_

#include <sstream>
#include <string>

typedef struct {
	bool enableIdentifiedPoints = true;
	bool enableInverseConic = true;
	bool enableRayMistrust = false;
	bool enablePointsPositionUpdate;
	bool enableUnusedVertexRemoving = false;
	bool enableMeshSaving = true;
	bool enableMeshPublishing = false;
	bool generateColoredMesh = false;

	float freeVoteThreshold = 1.0;
	int nonConicFreeVoteThreshold = 1;
	float rayRemovalThreshold = 100.0;
	float unusedVertexRemovalThreshold = 100.0;
	int primaryPointsVisibilityThreshold = 1;
	float outlierFilteringThreshold = 0.25;
	float minDistancePointPositionUpdate = 0.0;

	int maxPointsPerCamera = 1000;
	float maxDistanceCameraPoints = 40.0;
	float steinerGridStepLength = 10.0;
	int maxSteinerVerticesPerTriangle = 4;
	float w_1 = 1.0;
	float w_2 = 0.8;
	float w_3 = 0.4;
	float w_m = 1.0;

	int triangulationUpdateEvery = 10;
	int initialTriangulationUpdateSkip = 2;
	int saveMeshEvery = 10;

	int fakePointsMultiplier = 0;
	bool timeStatsOutput = false;
	bool debugOutput = false;
	bool publishReceivedPointcloud = false;
	bool publishUsedPointcloud = false;
	bool checkIntegrityWhenFinished = false;

	std::string inputTopic;
	std::string outputTopic;

	std::string receivedPointcloudTopic;
	std::string usedPointcloudTopic;

	std::string outputFolder = "res/output/";
	std::string timeStatsFolder = "res/output/";
	std::string countStatsFolder = "res/output/";

	std::string statsId = "";

	std::string toString() {
		std::stringstream out;
		out << "enableIdentifiedPoints: " << enableIdentifiedPoints << std::endl;
		out << "enableInverseConic: " << enableInverseConic << std::endl;
		out << "enableRayMistrust: " << enableRayMistrust << std::endl;
		out << "enablePointsPositionUpdate: " << enablePointsPositionUpdate << std::endl;
		out << "enableUnusedVertexRemoving: " << enableUnusedVertexRemoving << std::endl;
		out << "generateColoredMesh: " << generateColoredMesh << std::endl;

		out << "freeVoteThreshold: " << freeVoteThreshold << std::endl;
		out << "nonConicFreeVoteThreshold: " << nonConicFreeVoteThreshold << std::endl;
		out << "rayRemovalThreshold: " << rayRemovalThreshold << std::endl;
		out << "unusedVertexRemovalThreshold: " << unusedVertexRemovalThreshold << std::endl;
		out << "primaryPointsVisibilityThreshold: " << primaryPointsVisibilityThreshold << std::endl;
		out << "minDistancePointPositionUpdate: " << minDistancePointPositionUpdate << std::endl;

		out << "maxPointsPerCamera: " << maxPointsPerCamera << std::endl;
		out << "maxDistanceCameraPoints: " << maxDistanceCameraPoints << std::endl;
		out << "steinerGridStepLength: " << steinerGridStepLength << std::endl;
		out << "maxSteinerVerticesPerTriangle: " << maxSteinerVerticesPerTriangle << std::endl;
		out << "w_1: " << w_1 << std::endl;
		out << "w_2: " << w_2 << std::endl;
		out << "w_3: " << w_3 << std::endl;
		out << "w_m: " << w_m << std::endl;

		out << "triangulationUpdateEvery: " << triangulationUpdateEvery << std::endl;
		out << "initialTriangulationUpdateSkip: " << initialTriangulationUpdateSkip << std::endl;
		out << "saveMeshEvery: " << saveMeshEvery << std::endl;

		out << "fakePointsMultiplier: " << fakePointsMultiplier << std::endl;
		out << "timeStatsOutput: " << timeStatsOutput << std::endl;
		out << "debugOutput: " << debugOutput << std::endl;
		out << "publishReceivedPointcloud: " << publishReceivedPointcloud << std::endl;
		out << "publishUsedPointcloud: " << publishUsedPointcloud << std::endl;
		out << "checkIntegrityWhenFinished: " << checkIntegrityWhenFinished << std::endl;

		out << "outputFolder: " << outputFolder << std::endl;
		out << "timeStatsFolder: " << timeStatsFolder << std::endl;
		out << "countStatsFolder: " << countStatsFolder << std::endl;

		out << "statsId: " << statsId << std::endl;


		return out.str();
	}

} ManifoldReconstructionConfig;

#endif /* TYPES_TEST_HPP_ */
