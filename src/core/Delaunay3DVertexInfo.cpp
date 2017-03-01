/*
 * Delaunay3DVertexInfo.cpp
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#include <Delaunay3DVertexInfo.h>

Delaunay3DVertexInfo::Delaunay3DVertexInfo() {
}

Delaunay3DVertexInfo::Delaunay3DVertexInfo(long int pointId) :
		pointId_(pointId) {
}

Delaunay3DVertexInfo::~Delaunay3DVertexInfo() {
}

long int Delaunay3DVertexInfo::getPointId() const {
	return pointId_;
}

void Delaunay3DVertexInfo::setPointId(long int pointId) {
	if (pointId_ == -1) pointId_ = pointId;
}
