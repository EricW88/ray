#include "trimesh.h"
#include <assert.h>
#include <float.h>
#include <string.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include "../ui/TraceUI.h"
extern TraceUI* traceUI;

// using namespace std;

Trimesh::~Trimesh()
{
	for (auto m : materials)
		delete m;
	for (auto f : faces)
		delete f;
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex(const glm::dvec3& v)
{
	vertices.emplace_back(v);
}

void Trimesh::addMaterial(Material* m)
{
	materials.emplace_back(m);
}

void Trimesh::addNormal(const glm::dvec3& n)
{
	normals.emplace_back(n);
}

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace(int a, int b, int c)
{
	int vcnt = vertices.size();

	if (a >= vcnt || b >= vcnt || c >= vcnt)
		return false;

	TrimeshFace* newFace = new TrimeshFace(
	        scene, new Material(*this->material), this, a, b, c);
	newFace->setTransform(this->transform);
	if (!newFace->degen)
		faces.push_back(newFace);
	else
		delete newFace;

	// Don't add faces to the scene's object list so we can cull by bounding
	// box
	return true;
}

// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
const char* Trimesh::doubleCheck()
{
	if (!materials.empty() && materials.size() != vertices.size())
		return "Bad Trimesh: Wrong number of materials.";
	if (!normals.empty() && normals.size() != vertices.size())
		return "Bad Trimesh: Wrong number of normals.";

	return 0;
}

bool Trimesh::intersectLocal(ray& r, isect& i) const
{
	bool have_one = false;
	for (auto face : faces) {
		isect cur;
		if (face->intersect(r, cur)) {
			if (!have_one || (cur.getT() < i.getT())) {
				i = cur;
				have_one = true;
			}
		}
	}
	if (!have_one)
		i.setT(1000.0);
	return have_one;
}

bool TrimeshFace::intersect(ray& r, isect& i) const
{
	// std::cout << "running intersect!" << std::endl;

	return intersectLocal(r, i);

	// if(!traceUI->kdSwitch()) {
	// 	return intersectLocal(r, i);
	// }

	// KdTree<Geometry> *kdTree = this->getScene()->getKdTree();
	// double tmin;
	// double tmax;
	// bool found_one;
	// if(kdTree->findIntersection(r, i, tmin, tmax, found_one)) {
	// 	// std::cout << "running narrow phase" << std::endl;
	// 	return intersectLocal(r, i);
	// }
	// // std::cout << "failed broadphase" << std::endl;
	// return false;
}

// Intersect ray r with the triangle abc.  If it hits returns true,
// and put the parameter in t and the barycentric coordinates of the
// intersection in u (alpha) and v (beta).
bool TrimeshFace::intersectLocal(ray& r, isect& i) const
{
	// YOUR CODE HERE
	//
	// FIXME: Add ray-trimesh intersection
	// return false;
	glm::dvec3 a_coords = parent->vertices[ids[0]];
	glm::dvec3 b_coords = parent->vertices[ids[1]];
	glm::dvec3 c_coords = parent->vertices[ids[2]];

	// glm::dvec3 vab = (b_coords - a_coords);
	// glm::dvec3 vac = (c_coords - a_coords);
	// glm::dvec3 vcb = (b_coords - c_coords);

	double t = -1 * (glm::dot(this->normal, r.getPosition()) - this->dist) / glm::dot(this->normal, r.getDirection());
	// std::cout << t << std::endl;
	if(t < 0) {
		// std::cout << "no intersection" << std::endl;
		return false;
	}
	glm::dvec3 q = r.at(t);
	
	glm::dvec3 vec1 = glm::cross(b_coords - a_coords, q - a_coords);
	glm::dvec3 vec2 = glm::cross(c_coords - b_coords, q - b_coords);
	glm::dvec3 vec3 = glm::cross(a_coords - c_coords, q - c_coords);

	if(glm::dot(vec1, this->normal) < 0 || glm::dot(vec2, this->normal) < 0 || glm::dot(vec3, this->normal) < 0) {
		return false;
	}

	double area = glm::length(glm::cross(c_coords - a_coords, b_coords - a_coords));
	double alpha = glm::length(glm::cross(c_coords - b_coords, q - b_coords)) / area;
	double beta = glm::length(glm::cross(a_coords - c_coords, q - c_coords)) / area;
	double gamma = glm::length(glm::cross(b_coords - a_coords, q - a_coords)) / area;

	Material m;
	if(parent->vertNorms) {
		glm::dvec3 alphaNorm = parent->normals[ids[0]] * alpha;
		glm::dvec3 betaNorm = parent->normals[ids[1]] * beta;
		glm::dvec3 gammaNorm = parent->normals[ids[2]] * gamma;
		i.setN(alphaNorm + betaNorm + gammaNorm);
	} else {
		i.setN(this->normal);
	}
	if(parent->materials.empty()) {
		m = this->getMaterial();
	} else {
		m += alpha * (*parent->materials[ids[0]]);
		m += beta * (*parent->materials[ids[1]]);
		m += gamma * (*parent->materials[ids[2]]);
	}
	// 	m += alpha * (*parent->materials[ids[0]]);
	// 	m += beta * (*parent->materials[ids[1]]);
	// 	m += gamma * (*parent->materials[ids[2]]);
	// } else {
	// 	i.setN(this->normal);
	// 	m = this->getMaterial();
	// }
	i.setT(t);
	i.setObject(this);
	i.setMaterial(m);
	i.setUVCoordinates(glm::dvec2(alpha, beta));
	return true;
}

// Once all the verts and faces are loaded, per vertex normals can be
// generated by averaging the normals of the neighboring faces.
void Trimesh::generateNormals()
{
	int cnt = vertices.size();
	normals.resize(cnt);
	std::vector<int> numFaces(cnt, 0);

	for (auto face : faces) {
		glm::dvec3 faceNormal = face->getNormal();

		for (int i = 0; i < 3; ++i) {
			normals[(*face)[i]] += faceNormal;
			++numFaces[(*face)[i]];
		}
	}

	for (int i = 0; i < cnt; ++i) {
		if (numFaces[i])
			normals[i] /= numFaces[i];
	}

	vertNorms = true;
}

