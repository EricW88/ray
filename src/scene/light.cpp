#include <cmath>
#include <iostream>

#include "light.h"
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>


using namespace std;

double DirectionalLight::distanceAttenuation(const glm::dvec3& P) const
{
	// distance to light is infinite, so f(di) goes to 0.  Return 1.
	return 1.0;
}


glm::dvec3 DirectionalLight::shadowAttenuation(const ray& r, const glm::dvec3& p) const
{
	// YOUR CODE HERE:
	// You should implement shadow-handling code here.

	isect i;
	ray lightRay(p, getDirection(p), glm::dvec3(1, 1, 1));
	bool result = scene->intersect(lightRay, i);
	if(!result) {
		// std::cout << "no object hit" << std::endl;
		return glm::dvec3(1, 1, 1);
	}
	return glm::dvec3(0, 0, 0);
}

glm::dvec3 DirectionalLight::getColor() const
{
	return color;
}

glm::dvec3 DirectionalLight::getDirection(const glm::dvec3& P) const
{
	return -orientation;
}

double PointLight::distanceAttenuation(const glm::dvec3& P) const
{

	// YOUR CODE HERE

	// You'll need to modify this method to attenuate the intensity 
	// of the light based on the distance between the source and the 
	// point P.  For now, we assume no attenuation and just return 1.0
	// std::cout << "point light" << std::endl;

	// double length = glm::length(P - position);
	// double temp = 1 / (constantTerm + linearTerm * length + quadraticTerm * length * length);
	// std::cout << temp << " " << length << std::endl;
	// return 1 / (constantTerm + linearTerm * length + quadraticTerm * length * length);
	double length = glm::length(P - position);
	return std::min(1.0, 1 / (constantTerm + linearTerm * length + quadraticTerm * length * length));
}

glm::dvec3 PointLight::getColor() const
{
	return color;
}

glm::dvec3 PointLight::getDirection(const glm::dvec3& P) const
{
	return glm::normalize(position - P);
}


glm::dvec3 PointLight::shadowAttenuation(const ray& r, const glm::dvec3& p) const
{
	// YOUR CODE HERE:
	// You should implement shadow-handling code here.
	
	isect i;
	ray lightRay(p, getDirection(p), glm::dvec3(1, 1, 1));
	bool result = scene->intersect(lightRay, i);
	if(!result) {
		// std::cout << "no object hit" << std::endl;
		return glm::dvec3(1, 1, 1);
	}

	double lightDistance = glm::length(position - p);
	if(i.getT() < lightDistance) {
		// std::cout << "object behind" << std::endl;
		return glm::dvec3(0, 0, 0);
	}
	// std::cout << "object ahead" << std::endl;
	return glm::dvec3(1, 1, 1);
}

#define VERBOSE 0

