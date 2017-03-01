/*
 * ConfigParser.cpp
 *
 *  Created on: 18 May 2016
 *      Author: enrico
 */

#include "ConfigParser.h"
#include <Exceptions.hpp>
#include <stdexcept>
#include <rapidjson/reader.h>

ConfigParser::ConfigParser() {
	// TODO Auto-generated constructor stub

}

ConfigParser::~ConfigParser() {
	// TODO Auto-generated destructor stub
}

ManifoldReconstructionConfig ConfigParser::parse(std::string path) {

	rapidjson::Document document;
	std::ifstream fileStream;
	ManifoldReconstructionConfig c;

	fileStream.open(path.c_str());
	std::string str((std::istreambuf_iterator<char>(fileStream)), std::istreambuf_iterator<char>());
	document.Parse(str.c_str());

//	try {

		if (document.HasMember("inverseConicEnabled")) {
			if (!document["inverseConicEnabled"].IsBool())
				throw JsonAccessException("inverseConicEnabled is not a bool");
			c.enableInverseConic = document["inverseConicEnabled"].GetBool();
		}

		if (!document.HasMember("maxDistanceCamFeature"))
			throw JsonAccessException("maxDistanceCamFeature must be specified in the configuration");
		if (!document["maxDistanceCamFeature"].IsFloat())
			throw JsonAccessException("maxDistanceCamFeature is not a float");
		c.maxDistanceCameraPoints = document["maxDistanceCamFeature"].GetFloat();

		if (!document.HasMember("freeVoteThreshold"))
			throw JsonAccessException("freeVoteThreshold must be specified in the configuration");
		if (!document["freeVoteThreshold"].IsFloat())
			throw JsonAccessException("freeVoteThreshold is not a float");
		c.freeVoteThreshold = document["freeVoteThreshold"].GetFloat();

		if (!document.HasMember("w_1"))
			throw JsonAccessException("w_1 must be specified in the configuration");
		if (!document["w_1"].IsFloat())
			throw JsonAccessException("w_1");
		c.w_1 = document["w_1"].GetFloat();

		if (!document.HasMember("w_2"))
			throw JsonAccessException("w_2 must be specified in the configuration");
		if (!document["w_2"].IsFloat())
			throw JsonAccessException("w_2");
		c.w_2 = document["w_2"].GetFloat();

		if (!document.HasMember("w_3"))
			throw JsonAccessException("w_3 must be specified in the configuration");
		if (!document["w_3"].IsFloat())
			throw JsonAccessException("w_3");
		c.w_3 = document["w_3"].GetFloat();

		if (document.HasMember("w_m")) {
			if (!document["w_m"].IsFloat())
				throw JsonAccessException("w_m is not a float");
			c.w_m = document["w_m"].GetFloat();
		}

		if (document.HasMember("rayRemovalThreshold")) {
			if (!document["rayRemovalThreshold"].IsFloat())
				throw JsonAccessException("rayRemovalThreshold");
			c.rayRemovalThreshold = document["rayRemovalThreshold"].GetFloat();
		}

		if (document.HasMember("vertexRemovalThreshold")) {
			if (!document["vertexRemovalThreshold"].IsFloat())
				throw JsonAccessException("vertexRemovalThreshold is not a float");
			c.unusedVertexRemovalThreshold = document["vertexRemovalThreshold"].GetFloat();
		}

		if (document.HasMember("outlierFilteringThreshold")) {
			if (!document["outlierFilteringThreshold"].IsFloat())
				throw JsonAccessException("outlierFilteringThreshold is not a float");
			c.outlierFilteringThreshold = document["outlierFilteringThreshold"].GetFloat();
		}

		if (!document.HasMember("maxPointsPerCamera"))
			throw JsonAccessException("maxPointsPerCamera must be specified in the configuration");
		if (!document["maxPointsPerCamera"].IsInt())
			throw JsonAccessException("maxPointsPerCamera is not an int");
		c.maxPointsPerCamera = document["maxPointsPerCamera"].GetInt();

		if (document.HasMember("minDistancePointPositionUpdate")) {
			if (!document["minDistancePointPositionUpdate"].IsFloat())
				throw JsonAccessException("minDistancePointPositionUpdate is not a float");
			c.minDistancePointPositionUpdate = document["minDistancePointPositionUpdate"].GetFloat();
		}

		if (document.HasMember("enableRayMistrust")) {
			if (!document["enableRayMistrust"].IsBool())
				throw JsonAccessException("enableRayMistrust is not a bool");
			c.enableRayMistrust = document["enableRayMistrust"].GetBool();
		}

		if (document.HasMember("maxSteinerVerticesPerTriangle")) {
			if (!document["maxSteinerVerticesPerTriangle"].IsInt())
				throw JsonAccessException("maxSteinerVerticesPerTriangle is not an int");
			c.maxSteinerVerticesPerTriangle = document["maxSteinerVerticesPerTriangle"].GetInt();
		}
		
		if (!document.HasMember("steinerGridStepLength"))
			throw JsonAccessException("steinerGridStepLength must be specified in the configuration");
		if (!document["steinerGridStepLength"].IsFloat())
			throw JsonAccessException("steinerGridStepLength is not a float");
		c.steinerGridStepLength = document["steinerGridStepLength"].GetFloat();

		if (!document.HasMember("manifold_update_every"))
			throw JsonAccessException("manifold_update_every must be specified in the configuration");
		if (!document["manifold_update_every"].IsInt())
			throw JsonAccessException("manifold_update_every is not an int");
		c.triangulationUpdateEvery = document["manifold_update_every"].GetInt();

		if (document.HasMember("initial_manifold_update_skip")) {
			if (!document["initial_manifold_update_skip"].IsInt())
				throw JsonAccessException("initial_manifold_update_skip is not an int");
			c.initialTriangulationUpdateSkip = document["initial_manifold_update_skip"].GetInt();
		}

		if (document.HasMember("save_manifold_every")) {
			if (!document["save_manifold_every"].IsInt())
				throw JsonAccessException("save_manifold_every is not an int");
			c.saveMeshEvery = document["save_manifold_every"].GetInt();
		}

		if (document.HasMember("primary_points_visibility_threshold")) {
			if (!document["primary_points_visibility_threshold"].IsInt())
				throw JsonAccessException("primary_points_visibility_threshold is not an int");
			c.primaryPointsVisibilityThreshold = document["primary_points_visibility_threshold"].GetInt();
		}

		if (document.HasMember("all_sort_of_output")) {
			if (!document["all_sort_of_output"].IsBool())
				throw JsonAccessException("all_sort_of_output is not a bool");
			c.debugOutput = document["all_sort_of_output"].GetBool();
		}

		if (document.HasMember("time_stats_output")) {
			if (!document["time_stats_output"].IsBool())
				throw JsonAccessException("time_stats_output is not a bool");
			c.timeStatsOutput = document["time_stats_output"].GetBool();
		}

		if (document.HasMember("fake_points_multiplier")) {
			if (!document["fake_points_multiplier"].IsInt())
				throw JsonAccessException("fake_points_multiplier is not an int");
			c.fakePointsMultiplier = document["fake_points_multiplier"].GetInt();
		}

		if (!document.HasMember("update_points_position"))
			throw JsonAccessException("update_points_position must be specified in the configuration");
		if (!document["update_points_position"].IsBool())
			throw JsonAccessException("update_points_position is not a bool");
		c.enablePointsPositionUpdate = document["update_points_position"].GetBool();

		if (!document.HasMember("output_folder"))
			throw JsonAccessException("output_folder must be specified in the configuration");
		if (!document["output_folder"].IsString())
			throw JsonAccessException("output_folder is not a string");
		c.outputFolder = document["output_folder"].GetString();

		if (document.HasMember("time_stats_folder")) {
			if (!document["time_stats_folder"].IsString())
				throw JsonAccessException("time_stats_folder is not a string");
			c.timeStatsFolder = document["time_stats_folder"].GetString();
		} else {
			c.timeStatsFolder = document["output_folder"].GetString();
		}

		if (document.HasMember("count_stats_folder")) {
			if (!document["count_stats_folder"].IsString())
				throw JsonAccessException("count_stats_folder is not a string");
			c.countStatsFolder = document["count_stats_folder"].GetString();
		} else {
			c.timeStatsFolder = document["output_folder"].GetString();
		}

		if (document.HasMember("statsId")) {
			if (!document["statsId"].IsString())
				throw JsonAccessException("statsId is not a string");
			c.statsId = document["statsId"].GetString();
		}

//	} catch (JsonAccessException& e) {
//		std::cerr << e.what() << std::endl;
//		exit
//	}

	return c;
}
