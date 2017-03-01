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

/**
 * Header-only file with various types, especially those related to the CGAL library
 */
#ifndef TYPES_RECONSTR_HPP_
#define TYPES_RECONSTR_HPP_

#include <string>
#include <glm.hpp>
#include <Eigen/Core>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_hierarchy_3.h>
#include <CGAL/Triangulation_cell_base_with_info_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/intersections.h>

#include <CGAL/algorithm.h>

#include <Delaunay3DCellInfo.h>
#include <Delaunay3DVertexInfo.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Triangle_3 Triangle;
typedef K::Segment_3 Segment;

typedef CGAL::Triangulation_vertex_base_with_info_3<Delaunay3DVertexInfo, K> Vb;

typedef CGAL::Triangulation_hierarchy_vertex_base_3<Vb> Vbh;
typedef CGAL::Triangulation_cell_base_with_info_3<Delaunay3DCellInfo, K> Cb;
typedef CGAL::Triangulation_data_structure_3<Vbh, Cb> Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds> Dt;
typedef CGAL::Triangulation_hierarchy_3<Dt> Delaunay3;
typedef Delaunay3::Point PointD3;
typedef Delaunay3::Vertex_handle Vertex3D_handle;

namespace std {
template<>
struct hash<Delaunay3::Vertex_handle> {
	size_t operator()(Delaunay3::Vertex_handle const & x) const noexcept
	{
		return (std::hash<int>()(x->info().getPointId())); // TODO assign a unique id to Steiner poits also
	}
};
}
//
//Delaunay3::Vertex_handle v;
//
//int a = v.m_ptr.p;

struct index3 {
	int i, j, k;

	index3(int i_, int j_, int k_) {
		i = i_;
		j = j_;
		k = k_;
	}

	bool operator==(const index3& b) const {
		return i == b.i && j == b.j && k == b.k;
	}
	bool operator<(const index3& b) const {
		return i < b.i || (i == b.i && (j < b.j || (j == b.j && (k < b.k))));
	}

};

struct PointType;

struct CameraType {
	long unsigned int idCam;
	long int idReconstruction = -1;

	glm::mat3 intrinsics;
	glm::mat3 rotation;
	glm::vec3 translation;
	glm::mat4 cameraMatrix;
	glm::vec3 center;
	glm::mat4 mvp;

	std::string pathImage;

	int imageWidth;
	int imageHeight;

//	std::set<int> visiblePoints;
	std::set<PointType*> visiblePointsT, erasedPoints;

	void addPoint(PointType* point) {
		visiblePointsT.insert(point);
	}
};

struct PointType {
	long unsigned int idPoint;
	long int idReconstruction = -1;

	glm::vec3 position;
	std::set<CameraType*> viewingCams;

	float r = 0, g = 0, b = 0, a = 0;

	int getNumberObservations() {
		return viewingCams.size();
	}

	void addCamera(CameraType *cam) {
		viewingCams.insert(cam);
	}
};

struct sortTetByIntersection {
	inline bool operator()(const Delaunay3::Cell_handle& i, const Delaunay3::Cell_handle& j) {
		return i->info().getFreeVote() < j->info().getFreeVote();
	}
};

struct sortTetByIntersectionAndDefaultLess {
	inline bool operator()(const Delaunay3::Cell_handle& i, const Delaunay3::Cell_handle& j) {
		return (i->info().getFreeVote() < j->info().getFreeVote()) || ((i->info().getFreeVote() == j->info().getFreeVote()) && i < j);
	}
};

struct PointReconstruction {
	int idReconstruction;
	PointD3 position;

	float r = 0, g = 0, b = 0, a = 0;

	// true when the point has been moved and newPosition is set
	bool toBeMoved = false;
	PointD3 newPosition;

	// true when not yet in the triangulation (and vertexHandle isn't set)
	bool notTriangulated;
	Vertex3D_handle vertexHandle;

	std::vector<int> viewingCams;

	PointReconstruction() {
		position = PointD3(0.0, 0.0, 0.0);
		newPosition = PointD3(0.0, 0.0, 0.0);
		notTriangulated = true;
		idReconstruction = -1;
	}
};

struct CamReconstruction {
	int idReconstruction;
	PointD3 position;

	// true when the camera has been moved and newPosition is set
	bool toBeMoved = false;
	PointD3 newPosition;

	Vertex3D_handle vertexHandle;

	std::vector<int> visiblePoints;
	std::vector<int> newVisiblePoints;

	CamReconstruction() {
		position = PointD3(0.0, 0.0, 0.0);
		idReconstruction = -1;
	}
};

#endif /* TYPES_HPP_ */
