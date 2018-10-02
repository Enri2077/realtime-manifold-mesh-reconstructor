//  Copyright 2014 Andrea Romanoni
//
//  This file is part of manifoldReconstructor.
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

#include <cstdlib>
#include <iostream>
#include <map>
#include <set>
#include <utility>

#include <CameraPointsCollection.h>
#include <Chronometer.h>
#include <Logger.h>
#include <ReconstructFromSLAMData.h>
#include <OpenMvgParser.h>
#include <ConfigParser.h>
#include <types_config.hpp>
#include <types_reconstructor.hpp>

#include <points_filtering.hpp>


void printUsage(char *name);
void readArgs(int argc, char **argv);

int maxIterations_ = 0;
std::string input_file;
std::string config_file;


int main(int argc, char **argv)
{

    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    readArgs(argc, argv);

    std::cout << "input set to: " << input_file << std::endl;
    std::cout << "config set to: " << config_file << std::endl;
    std::cout << "max_iterations set to: " << maxIterations_ << std::endl;



    utilities::Logger log;
    std::ofstream statsFile;
    std::ofstream visiblePointsFile;

    ManifoldReconstructionConfig confManif;
    ConfigParser configParser = ConfigParser();
    confManif = configParser.parse(config_file);

    SfMData sfm_data_;

    log.startEvent();


    CameraPointsCollection incData;
    OpenMvgParser op_openmvg(argv[1]);
    op_openmvg.parse();

    ReconstructFromSLAMData m(confManif);

    m.setExpectedTotalIterationsNumber((maxIterations_) ? maxIterations_ + 1 : op_openmvg.getSfmData().numCameras_);

    sfm_data_ = op_openmvg.getSfmData();


    std::vector<bool> inliers;
    outlierFiltering(inliers, confManif.outlierFilteringThreshold, sfm_data_);

    for (int cameraIndex = 0; cameraIndex < sfm_data_.camerasList_.size(); cameraIndex++) {
        CameraType* camera = &sfm_data_.camerasList_[cameraIndex];
        camera->idCam = cameraIndex;
        incData.addCamera(camera);
    }

    for (int pointIndex = 0; pointIndex < sfm_data_.points_.size(); pointIndex++) {
        if (inliers[pointIndex]) {
            PointType* point = new PointType();
            point->idPoint = pointIndex;
            point->position = sfm_data_.points_[pointIndex];

            incData.addPoint(point);
        }
    }

    for (int cameraIndex = 0; cameraIndex < sfm_data_.camerasList_.size(); cameraIndex++) {

        int inliersCount = 0, inlierPointIndex = 0;
        for (auto pointIndex : sfm_data_.pointsVisibleFromCamN_[cameraIndex])
            if (inliers[pointIndex]) inliersCount++;

        for (auto pointIndex : sfm_data_.pointsVisibleFromCamN_[cameraIndex]) {
            if (confManif.maxPointsPerCamera < inliersCount) {
                if (inliers[pointIndex]) {

                    if (inlierPointIndex < confManif.maxPointsPerCamera){
                        incData.addVisibility(cameraIndex, pointIndex);
                    }

                    inlierPointIndex++;
                }

            } else {
                if (inliers[pointIndex]) incData.addVisibility(cameraIndex, pointIndex);
            }
        }

    }

    // Main loop
    for (auto index_camera : incData.getCameras()) {
        CameraType* camera = (index_camera.second);

        if (camera == NULL) {
            continue;
        }

        // If maxIterations_ is set, only execute ReconstructFromSLAMData::addCamera maxIterations_ times
        if (maxIterations_ && m.iterationCount >= maxIterations_) {
            break;
        }

        log.startEvent();

        m.addCamera(camera);

        // Skip the manifold update for the first confManif.initial_manifold_update_skip cameras
        if (m.iterationCount > confManif.initialTriangulationUpdateSkip && !(m.iterationCount % confManif.triangulationUpdateEvery)) {
            m.update();
        }

        if (m.iterationCount > confManif.initialTriangulationUpdateSkip && !(m.iterationCount % confManif.triangulationUpdateEvery)) {
            m.integrityCheck();
        }

        if (m.iterationCount && !(m.iterationCount % confManif.saveMeshEvery)) {
            int manifold_seq = m.iterationCount / confManif.saveMeshEvery;
            m.saveMesh("output/", "current_" + std::to_string(manifold_seq));
        }

        log.endEventAndPrint("main loop\t\t\t\t\t\t", true);

        if (m.iterationCount > confManif.initialTriangulationUpdateSkip && !(m.iterationCount % confManif.triangulationUpdateEvery)) {
            m.insertStatValue(log.getLastDelta());
        }
    }

    // Do a last manifold update in case op.numCameras() isn't a multiple of confManif.manifold_update_every
    if (m.iterationCount > confManif.initialTriangulationUpdateSkip) {
        m.update();
    }

    m.saveMesh("output/", "final");

    log.endEventAndPrint("main\t\t\t\t\t\t", true);

    return 0;


}

void printUsage(char *name)
{
    std::cout << name << " sfm_data.json [config_file.json [max_iterations]]" << std::endl;
}


void readArgs(int argc, char **argv)
{
    if (argc == 4) {
        maxIterations_ = atoi(argv[3]);
        input_file = argv[1];
        config_file = argv[2];
    } else if (argc == 3) {
        input_file = argv[1];
        config_file = argv[2];
    } else if (argc == 2) {
        input_file = argv[1];
        config_file = "res/config/default.json";
    }
}
