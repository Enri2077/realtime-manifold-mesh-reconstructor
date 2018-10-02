#ifndef MAIN_POINTS_FILTERING_H_
#define MAIN_POINTS_FILTERING_H_


#include <CameraPointsCollection.h>
#include <Chronometer.h>
#include <Logger.h>
#include <ORBIncrementalParser.h>
#include <ReconstructFromSLAMData.h>
#include <OpenMvgParser.h>
#include <ReconstructFromSfMData.h>
#include <ConfigParser.h>
#include <types_config.hpp>
#include <types_reconstructor.hpp>
#include <cstdlib>
#include <iostream>
#include <map>
#include <set>
#include <utility>


int point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint, cv::Mat &J, cv::Mat &hessian);

int GaussNewton(const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points, cv::Point3f init3Dpoint, cv::Point3f &optimizedPoint, const float outlierThreshold);

void outlierFiltering(std::vector<bool>& inliers, const float outlierThreshold, SfMData &sfm_data_);


#endif // MAIN_POINTS_FILTERING_H_
