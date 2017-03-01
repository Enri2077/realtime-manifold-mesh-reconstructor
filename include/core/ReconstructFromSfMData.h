/*
 * ReconstructFromSfMData.h
 *
 *  Created on: 17 mar 2016
 *      Author: andrea
 */

#ifndef RECONSTRUCTFROMSFMDATA_H_
#define RECONSTRUCTFROMSFMDATA_H_

#include <SfMData.h>
#include <types_config.hpp>
#include <TriangulationManager.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class ReconstructFromSfMData {
public:
	ReconstructFromSfMData(const SfMData &sfm_data, ManifoldReconstructionConfig& manifConf);
	virtual ~ReconstructFromSfMData();
	void run();
	void overwriteFocalY(float f);
private:

	void outlierFiltering(std::vector<bool>& inliers);
	int GaussNewton(const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points, cv::Point3f init3Dpoint, cv::Point3f &optimizedPoint);

	int point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint, cv::Mat &jacobian, cv::Mat &hessian);

	SfMData sfm_data_;
	ManifoldReconstructionConfig manifConf_;
	TriangulationManager* manifRec_;
};

#endif /* RECONSTRUCTFROMSFMDATA_H_ */
