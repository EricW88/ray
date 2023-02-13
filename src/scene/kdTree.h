#pragma once

#include <float.h>
#include <glm/glm.hpp>
#include <vector>
#include <iostream>

#include "bbox.h"
#include "ray.h"
// Note: you can put kd-tree here

// class BoundingBox;
class ray;
class isect;

class Plane {
    public:
        Plane() : axis(glm::dvec3(0, 0, 0)), position(glm::dvec3(0, 0, 0)), left(false) {}
        Plane(glm::dvec3 a, glm::dvec3 p, bool l) : axis(a), position(p), left(l) {}

        glm::dvec3 getAxis() {return axis;}
        glm::dvec3 getPosition() {return position;}

        bool isLeft() {return left;}

        bool operator < (const Plane& other) const{return glm::dot(axis, position) < glm::dot(other.axis, other.position);}
    private:
        glm::dvec3 axis;
        glm::dvec3 position;
        bool left;
};

template <typename Objects>
class KdTree {
    public:
        static KdTree<Objects> *buildTree(std::vector<Objects*> objList, BoundingBox bbox, int depth, int leafSize);
        static Plane findBestSplitPlane(std::vector<Objects *> objList, BoundingBox bbox);


        KdTree(KdTree<Objects> *l, KdTree<Objects> *r, BoundingBox box) : left(l), right(r), bbox(box) {}
        // KdTree(std::vector<Objects *> objList, BoundingBox box, int depth, int leafSize);
        // Plane findBestSplitPlane(std::vector<Objects *> objList, BoundingBox bbox);
        int countObjects(std::vector<Objects *> objList, BoundingBox bbox);

        virtual void findIntersection(ray &r, isect &i, double tmin, double tmax, bool &found_one) = 0;

        BoundingBox getBoundingBox() {return bbox;}
        KdTree<Objects>* getLeft() {return left;}
        KdTree<Objects>* getRight() {return right;}

    protected:
        KdTree<Objects> *left;
        KdTree<Objects> *right;
        BoundingBox bbox;

};



template<typename Objects>
class SplitNode : public KdTree<Objects> {
    public:
        SplitNode(Plane pl, KdTree<Objects> *l, KdTree<Objects> *r, BoundingBox box) : plane(pl), KdTree<Objects>(l, r, box) {}
        void findIntersection(ray &r, isect &i, double tmin, double tmax, bool &found_one);
    private:
        Plane plane;

    
};

template<typename Objects>
class LeafNode : public KdTree<Objects> {
    public:
        LeafNode(std::vector<Objects *> o, BoundingBox box) : objList(o), KdTree<Objects>(NULL, NULL, box) {}
        void findIntersection(ray &r, isect &i, double tmin, double tmax, bool &found_one);
    private:
        std::vector<Objects *> objList;
};
