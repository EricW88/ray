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
	glm::dvec3 d = r.getDirection();
	// std::cout << glm::to_string(d) << std::endl;
	double abs_x = abs(d.x);
	double abs_y = abs(d.y);
	double abs_z = abs(d.z);
	// std::cout << d.x << " " << d.y << " " << d.z << std::endl;
	if(abs_x > abs_y && abs_x > abs_z) {
		
		if(d.x > 0) {
			glm::dvec2 coord((-d.z / abs_x + 1) / 2, (d.y / abs_x + 1) / 2);
			return tMap[0]->getMappedValue(coord);
		} else {
			// std::cout << "neg x" << std::endl;

			glm::dvec2 coord((d.z / abs_x + 1) / 2, (d.y / abs_x + 1) / 2);
			// std::cout << coord.x << " " << coord.y << std::endl;
			return tMap[1]->getMappedValue(coord);
		}
	} else if (abs_y > abs_x && abs_y > abs_z) {
		// std::cout << "y" << std::endl;
		if(d.y > 0) {
			glm::dvec2 coord((d.x / abs_y + 1) / 2, (-d.z / abs_y + 1) / 2);
			return tMap[2]->getMappedValue(coord);
		} else {
			glm::dvec2 coord((d.x / abs_y + 1) / 2, (d.z / abs_y + 1) / 2);
			return tMap[3]->getMappedValue(coord);
		}
	} else {
		// std::cout << "z" << std::endl;
		if(d.z > 0) {
			glm::dvec2 coord((d.x / abs_z + 1) / 2, (d.y / abs_z + 1) / 2);
			return tMap[4]->getMappedValue(coord);
		} else {
			glm::dvec2 coord((-d.x / abs_z + 1) / 2, (d.y / abs_z + 1) / 2);
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
