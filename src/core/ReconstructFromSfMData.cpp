/*
 * ReconstructFromSfMData.cpp
 *
 *  Created on: 17 mar 2016
 *      Author: andrea
 */

#include <ReconstructFromSfMData.h>
//#include <CameraPointsCollection.h>
#include <utilities.hpp>
#include <Logger.h>
#include <set>
#include <map>

ReconstructFromSfMData::ReconstructFromSfMData(const SfMData &sfm_data, ManifoldReconstructionConfig& manifConf) {
	sfm_data_ = sfm_data;

	manifConf_ = manifConf;

	manifRec_ = new TriangulationManager(manifConf_);

}

ReconstructFromSfMData::~ReconstructFromSfMData() {
}

void ReconstructFromSfMData::run() {

	// Filter the points
	std::vector<bool> inliers;
	outlierFiltering(inliers);

	std::vector<glm::vec3> camCenters;
	std::vector<glm::vec3> points;
	std::vector<std::vector<int> > camViewingPointN;

	std::set<int> addedPoints;
	std::map<int, int> sfmToRecIndex;
	int currentRecIndex = 0;


//	CameraPointsCollection data;

//	for (auto c : sfm_data_.camerasList_) {
//		camCenters.push_back(c.center);
//	}
//
//	int count = 0;
//	for (int curpt = 0; curpt < sfm_data_.points_.size(); curpt++) {
//
//		if (inliers[curpt]) {
//			count++;
//			points.push_back(sfm_data_.points_[curpt]);
//			camViewingPointN.push_back(sfm_data_.camViewingPointN_[curpt]);
//		}
//	}

//	std::cout << "inlier ratio " << count << " on " << sfm_data_.points_.size() << " = " << static_cast<float>(count) / static_cast<float>(sfm_data_.points_.size()) << std::endl;

	// utilities::saveVisibilityPly(camCenters,points,camViewingPointN,"inliersOK",false);
	// utilities::saveVisibilityPly(sfm_data_, "sfm_data_");

//	for (auto camC : sfm_data_.camerasList_) {
//
////    manifRec_->addCameraCenter(camC.center.x, camC.center.y, camC.center.z);
//		manifRec_->addCamera(camC.center.x, camC.center.y, camC.center.z);
//	}

	// The points can be added all at once WRONG
//	for (auto p : sfm_data_.points_) manifRec_->addPoint(p.x, p.y, p.z);

//	for (int curCamIdx = 0; curCamIdx < sfm_data_.camerasList_.size(); curCamIdx += 1) {
//
//		for (auto curPtIdx : sfm_data_.pointsVisibleFromCamN_[curCamIdx]) {
//
//			if (sfm_data_.camViewingPointN_[curPtIdx].size() >= 2 && inliers[curPtIdx]) manifRec_->addVisibilityPair(curCamIdx, curPtIdx);
//		}
//	}





	for (int cameraIndex = 0; cameraIndex < sfm_data_.camerasList_.size(); cameraIndex++) {
		auto camera = sfm_data_.camerasList_[cameraIndex];

		// The cameras and their rays must be added just before being used to update the triangulation.
		// Rays relative to cameras added before the last update are ignored unless the camera is updated.

		manifRec_->addCamera(camera.center.x, camera.center.y, camera.center.z);
		std::cout << "Add camera " << cameraIndex << std::endl;

		// The points must be added only once

		for (auto pointIndex : sfm_data_.pointsVisibleFromCamN_[cameraIndex]){
			if (!addedPoints.count(pointIndex)){
				addedPoints.insert(pointIndex);
				sfmToRecIndex[pointIndex] = currentRecIndex++;

				auto p = sfm_data_.points_[pointIndex];
				manifRec_->addPoint(p.x, p.y, p.z);
			}
		}

		for(int pointIndex : sfm_data_.pointsVisibleFromCamN_[cameraIndex])
			if (sfm_data_.camViewingPointN_[pointIndex].size() >= 2 && inliers[pointIndex])
				manifRec_->addVisibilityPair(cameraIndex, sfmToRecIndex[pointIndex]);

		if (cameraIndex % 11 == 10) {
			utilities::Logger log;
			log.startEvent();

			manifRec_->updateTriangulation();

			log.endEventAndPrint("Update ", true);
			log.startEvent();

			std::stringstream s;
			s << "Manifold" << cameraIndex << ".off";
			manifRec_->saveManifold(s.str());

			log.endEventAndPrint("Export ", true);
		}
	}

	utilities::Logger log;
	log.startEvent();
//	manifRec_->growManifold();
//	manifRec_->growManifoldSev();
//	manifRec_->growManifold();
//	manifRec_->saveManifold("ManifoldFinal.off");
	manifRec_->updateTriangulation();
	manifRec_->saveManifold("ManifoldFinal.off");

	std::vector<int> age;

	for (int cur = 0; cur < sfm_data_.camerasList_.size(); cur++) {
		age.push_back(cur);
	}

//	manifRec_->saveFreespace("FreeFinal.off");
//
//	manifRec_->saveOldManifold("ManifoldWithoutSteinerPointsFinal.off", age);
	std::cout << "DONE" << std::endl;
	log.endEventAndPrint("Grow ", true);
}

void ReconstructFromSfMData::overwriteFocalY(float f) {
	for (auto c : sfm_data_.camerasList_) {
		c.intrinsics[1][1] = f;
	}

}

void ReconstructFromSfMData::outlierFiltering(std::vector<bool>& inliers) {

	inliers.assign(sfm_data_.points_.size(), false);
	std::vector<cv::Mat> cameras;
	std::vector<cv::Point2f> measures;
	cv::Point3f init3Dpoint;
	cv::Point3f optimizedPoint;

	for (int curPt3D = 0; curPt3D < sfm_data_.points_.size(); curPt3D++) {
		cameras.clear();
		cameras.assign(sfm_data_.camViewingPointN_[curPt3D].size(), cv::Mat());
		for (int curC = 0; curC < sfm_data_.camViewingPointN_[curPt3D].size(); curC++) {
			cameras[curC] = cv::Mat(4, 4, CV_32F);
			for (int row = 0; row < 4; row++) {
				for (int col = 0; col < 4; col++) {
					cameras[curC].at<float>(row, col) = sfm_data_.camerasList_[sfm_data_.camViewingPointN_[curPt3D][curC]].cameraMatrix[row][col];
				}
			}

		}

		measures.clear();
		measures.assign(sfm_data_.point2DoncamViewingPoint_[curPt3D].size(), cv::Point2f());
		for (int curMeas = 0; curMeas < sfm_data_.point2DoncamViewingPoint_[curPt3D].size(); curMeas++) {
			measures[curMeas].x = sfm_data_.point2DoncamViewingPoint_[curPt3D][curMeas].x;
			measures[curMeas].y = sfm_data_.point2DoncamViewingPoint_[curPt3D][curMeas].y;
		}

		init3Dpoint.x = sfm_data_.points_[curPt3D].x;
		init3Dpoint.y = sfm_data_.points_[curPt3D].y;
		init3Dpoint.z = sfm_data_.points_[curPt3D].z;

		if (GaussNewton(cameras, measures, init3Dpoint, optimizedPoint) != -1) {

			sfm_data_.points_[curPt3D].x = optimizedPoint.x;
			sfm_data_.points_[curPt3D].y = optimizedPoint.y;
			sfm_data_.points_[curPt3D].z = optimizedPoint.z;
			inliers[curPt3D] = true;
		}
	}

}

int ReconstructFromSfMData::GaussNewton(const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points, cv::Point3f init3Dpoint, cv::Point3f &optimizedPoint) {
	int numMeasures = points.size();
	cv::Mat r = cv::Mat(numMeasures * 2, 1, CV_32F);

	cv::Mat curEstimate3DPoint = cv::Mat(3, 1, CV_32F);
	cv::Mat curEstimate3DPointH = cv::Mat(4, 1, CV_32F);
	curEstimate3DPoint.at<float>(0, 0) = init3Dpoint.x;
	curEstimate3DPoint.at<float>(1, 0) = init3Dpoint.y;
	curEstimate3DPoint.at<float>(2, 0) = init3Dpoint.z;

	cv::Mat J, H;
	float last_mse = 0;
	int i;
	for (i = 0; i < 30; i++) {

		float mse = 0;
		//compute residuals
		for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
			curEstimate3DPointH.at<float>(0, 0) = curEstimate3DPoint.at<float>(0, 0);
			curEstimate3DPointH.at<float>(1, 0) = curEstimate3DPoint.at<float>(1, 0);
			curEstimate3DPointH.at<float>(2, 0) = curEstimate3DPoint.at<float>(2, 0);
			curEstimate3DPointH.at<float>(3, 0) = 1.0;
			cv::Mat cur2DpositionH = cameras[curMeas] * curEstimate3DPointH;

			r.at<float>(2 * curMeas, 0) = ((points[curMeas].x - cur2DpositionH.at<float>(0, 0) / cur2DpositionH.at<float>(2, 0)));
			mse += r.at<float>(2 * curMeas, 0) * r.at<float>(2 * curMeas, 0);

			r.at<float>(2 * curMeas + 1, 0) = ((points[curMeas].y - cur2DpositionH.at<float>(1, 0) / cur2DpositionH.at<float>(2, 0)));
			mse += r.at<float>(2 * curMeas + 1, 0) * r.at<float>(2 * curMeas + 1, 0);
#ifdef DEBUG_OPTIMIZATION_VERBOSE
			if(i==0) {
				std::cout<<"CurMeas: "<<curMeas<<std::endl<<"curEstimate3DPointH="<< curEstimate3DPointH.t()<<std::endl;
				std::cout<<"CurCam"<<cameras[curMeas]<<std::endl;
				std::cout<<"cur2DpositionH: "<<cur2DpositionH.at<float>(0, 0)/cur2DpositionH.at<float>(2, 0)<<", "<<cur2DpositionH.at<float>(1, 0)/cur2DpositionH.at<float>(2, 0)<<std::endl;
				std::cout<<"points[curMeas]: "<<points[curMeas]<<std::endl;
				std::cout<<"residual on x: "<<r.at<float>(2 * curMeas, 0)<<std::endl;
				std::cout<<"residual on y: "<<r.at<float>(2 * curMeas + 1 , 0)<<std::endl;
				std::cout<<std::endl;}
#endif
		}
		//mse = sqrt(mse)/(numMeasures*2);

		if (abs(mse / (numMeasures * 2) - last_mse) < 0.0000000005) {
			break;
		}
		last_mse = mse / (numMeasures * 2);

		if (point2D3DJacobian(cameras, curEstimate3DPoint, J, H) == -1) {
			//std::cout<<"It= "<<" point2D3DJacobian(cameras, curEstimate3DPoint, J, H) == -1 "<<std::endl;
			return -1;
		}
#ifdef DEBUG_OPTIMIZATION_VERBOSE
		std::cout<<"J: "<<J<<std::endl;
		std::cout<<"H: "<<H<<std::endl;
#endif

		curEstimate3DPoint += H.inv() * J.t() * r;

#ifdef DEBUG_OPTIMIZATION
		std::cout << "It= " << i << " last_mse " << last_mse << std::endl;
#endif
	}

	if (last_mse < 0.25/*3 pixels*/) {
		optimizedPoint.x = curEstimate3DPoint.at<float>(0, 0);
		optimizedPoint.y = curEstimate3DPoint.at<float>(1, 0);
		optimizedPoint.z = curEstimate3DPoint.at<float>(2, 0);
		return 1;
	} else {
		return -1;
	}
}

int ReconstructFromSfMData::point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint, cv::Mat &J, cv::Mat &hessian) {

	int numMeasures = cameras.size();
	cv::Mat cur3DPointHomog = cv::Mat(4, 1, CV_32F);
	;
	cur3DPointHomog.at<float>(0, 0) = cur3Dpoint.at<float>(0, 0);
	cur3DPointHomog.at<float>(1, 0) = cur3Dpoint.at<float>(1, 0);
	cur3DPointHomog.at<float>(2, 0) = cur3Dpoint.at<float>(2, 0);
	cur3DPointHomog.at<float>(3, 0) = 1.0;

	J = cv::Mat(2 * numMeasures, 3, CV_32FC1);  //2 rows for each point: one for x, the other for y
	hessian = cv::Mat(3, 3, CV_32FC1);

	/*std::cout << "gdevre" <<std::endl;
	 std::cout << cameras[0] <<std::endl;*/
	for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
		cv::Mat curReproj = cameras[curMeas] * cur3DPointHomog;
		float xH = curReproj.at<float>(0, 0);
		float yH = curReproj.at<float>(1, 0);
		float zH = curReproj.at<float>(2, 0);
		float p00 = cameras[curMeas].at<float>(0, 0);
		float p01 = cameras[curMeas].at<float>(0, 1);
		float p02 = cameras[curMeas].at<float>(0, 2);
		float p10 = cameras[curMeas].at<float>(1, 0);
		float p11 = cameras[curMeas].at<float>(1, 1);
		float p12 = cameras[curMeas].at<float>(1, 2);
		float p20 = cameras[curMeas].at<float>(2, 0);
		float p21 = cameras[curMeas].at<float>(2, 1);
		float p22 = cameras[curMeas].at<float>(2, 2);

		//d(P*X3D)/dX
		J.at<float>(2 * curMeas, 0) = (p00 * zH - p20 * xH) / (zH * zH);
		J.at<float>(2 * curMeas + 1, 0) = (p10 * zH - p20 * yH) / (zH * zH);

		//d(P*X3D)/dY
		J.at<float>(2 * curMeas, 1) = (p01 * zH - p21 * xH) / (zH * zH);
		J.at<float>(2 * curMeas + 1, 1) = (p11 * zH - p21 * yH) / (zH * zH);

		//d(P*X3D)/dZ
		J.at<float>(2 * curMeas, 2) = (p02 * zH - p22 * xH) / (zH * zH);
		J.at<float>(2 * curMeas + 1, 2) = (p12 * zH - p22 * yH) / (zH * zH);
	}

	hessian = J.t() * J;
	float d;
	d = cv::determinant(hessian);
	if (d < 0.0000000001) {
		//printf("doh");
		return -1;
	} else {
		return 1;
	}
}

