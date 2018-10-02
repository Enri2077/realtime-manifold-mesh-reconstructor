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


    if (confManif.timeStatsOutput) {
        log.startEvent();
    }

    ORBIncrementalParser op(argv[1], confManif);

    if (confManif.timeStatsOutput) {
        log.endEventAndPrint("Parsing\t\t\t\t", true);
    }

    ReconstructFromSLAMData m(confManif);
    m.setExpectedTotalIterationsNumber((maxIterations_) ? maxIterations_ + 1 : op.numCameras());

    // Main loop
    for (int i = 0; i < op.numCameras(); i++) {
        CameraType* camera = op.nextCamera();

        bool updatingCamera = camera->idReconstruction >= 0;

        if(updatingCamera) {
            continue;
        }

        // If maxIterations_ is set, only execute ReconstructFromSLAMData::addCamera maxIterations_ times
        if (maxIterations_ && m.iterationCount >= maxIterations_) {
            break;
        }

        log.startEvent();

        m.addCamera(camera);

        // Skip the manifold update for the first confManif.initialTriangulationUpdateSkip cameras
        if (!updatingCamera && m.iterationCount > confManif.initialTriangulationUpdateSkip && !(m.iterationCount % confManif.triangulationUpdateEvery)) {
            m.update();
        }

        if (!updatingCamera && m.iterationCount && !(m.iterationCount % confManif.saveMeshEvery)){
            std::stringstream ssm, ssnm;
            ssm << "current_" << m.iterationCount << "_" << confManif.statsId;
            ssnm << ssm.str() <<  "_NOT_MANIFOLD";

            if (!confManif.checkIntegrityWhenFinished || m.integrityCheck()) {
                m.saveMesh(confManif.outputFolder, ssm.str());
            } else {
                m.saveMesh(confManif.outputFolder, ssnm.str());
            }
        }

        log.endEventAndPrint("main loop\t\t\t\t\t\t", true);


        if (!updatingCamera && m.iterationCount > confManif.initialTriangulationUpdateSkip && !(m.iterationCount % confManif.triangulationUpdateEvery)) {
            m.insertStatValue(log.getLastDelta());
        }

    }

    // Do a last manifold update in case op.numCameras() isn't a multiple of confManif.manifold_update_every
    if (m.iterationCount > confManif.initialTriangulationUpdateSkip) {
        m.update();
    }

    m.saveMesh(confManif.outputFolder, confManif.statsId);

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
