/*
 * Ray.hpp
 *
 *  Created on: 29 Apr 2016
 *      Author: enrico
 */

#ifndef INCLUDE_MANIFOLDRECONSTRUCTOR_RAY_HPP_
#define INCLUDE_MANIFOLDRECONSTRUCTOR_RAY_HPP_


#include <types_reconstructor.hpp>
#include <set>

struct RayPath {
	bool valid = true;
	int cameraId = -1;
	int pointId = -1;

	double mistrustVote = 0.0;

	std::vector<Delaunay3::Cell_handle> path; // TODO could it be a vector?
};





#endif /* INCLUDE_MANIFOLDRECONSTRUCTOR_RAY_HPP_ */
