#include <iostream>

#include "cubeMap.h"
#include "ray.h"
#include "../ui/TraceUI.h"
#include "../scene/material.h"
extern TraceUI* traceUI;

glm::dvec3 CubeMap::getColor(ray r) const
{
	// YOUR CODE HERE
	// FIXME: Implement Cube Map here
	glm::dvec3 d = glm::normalize(r.getDirection());
	double x = abs(d.x);
	double y = abs(d.y);
	double z = abs(d.z);
	// std::cout << d.x << " " << d.y << " " << d.z << std::endl;
	if(x >= y && x >= z) {
		
		if(d.x > 0) {
			std::cout << "pos x" << std::endl;
			glm::dvec2 coord(d.y, -d.z);
			return tMap[0]->getMappedValue(coord);
		} else {
			// std::cout << "neg x" << std::endl;
			glm::dvec2 coord(d.z / d.x, d.y / d.x);
			return tMap[1]->getMappedValue(coord);
		}
	} else if (y >= x && y >= z) {
		std::cout << "y" << std::endl;
		if(d.y > 0) {
			glm::dvec2 coord(-d.z, d.x);
			return tMap[2]->getMappedValue(coord);
		} else {
			glm::dvec2 coord(d.z, d.x);
			return tMap[3]->getMappedValue(coord);
		}
	} else {
		std::cout << "z" << std::endl;
		if(d.z > 0) {
			glm::dvec2 coord(d.y, d.x);
			return tMap[4]->getMappedValue(coord);
		} else {
			glm::dvec2 coord(d.y, -d.x);
			return tMap[5]->getMappedValue(coord);
		}
	}
	// return glm::dvec3();
}

CubeMap::CubeMap()
{
}

CubeMap::~CubeMap()
{
}

void CubeMap::setNthMap(int n, TextureMap* m)
{
	if (m != tMap[n].get())
		tMap[n].reset(m);
}
