/*
 * CameraPointsCollection.cpp
 *
 *  Created on: 25 Mar 2016
 *      Author: enrico
 */

#include <CameraPointsCollection.h>
#include <types_reconstructor.hpp>
#include <map>
#include <utility>

CameraPointsCollection::CameraPointsCollection() {

}

CameraPointsCollection::~CameraPointsCollection() {
	clear();
}

void CameraPointsCollection::clear() {
	points_.clear();
	cameras_.clear();
}

int CameraPointsCollection::hasCamera(long unsigned int cameraId) {
	return cameras_.count(cameraId);
}

void CameraPointsCollection::addCamera(CameraType* cam) {
	cameras_.insert(std::pair<long unsigned int, CameraType*>(cam->idCam, cam));
}

CameraType* CameraPointsCollection::getCamera(long unsigned int camId) {
	return cameras_.at(camId);
}

int CameraPointsCollection::numCameras() {
	return cameras_.size();
}

int CameraPointsCollection::hasPoint(long unsigned int pointId) {
	return points_.count(pointId);
}

void CameraPointsCollection::addPoint(PointType* point) {
	points_.insert(std::pair<long unsigned int, PointType*>(point->idPoint, point));
}

PointType* CameraPointsCollection::getPoint(long unsigned int pointId) {
	return points_.at(pointId);
}

int CameraPointsCollection::numPoints() {
	return points_.size();
}

void CameraPointsCollection::addVisibility(long unsigned int camId, long unsigned int pointId) {
	//TODO check if camera and point are in cameras_ and points_
	CameraType* camera = getCamera(camId);
	PointType* point = getPoint(pointId);
	addVisibility(camera, point);
}
void CameraPointsCollection::addVisibility(CameraType* camera, PointType* point) {
	//TODO check if camera and point are in cameras_ and points_
	camera->addPoint(point);
	point->addCamera(camera);

}

