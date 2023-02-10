#include <float.h>

#pragma once

// Note: you can put kd-tree here

class Plane {
    public:
        Plane(glm::dvec3 a, glm::dvec3 p) : axis(a), position(p) {}
    private:
        glm::dvec3 axis;
        glm::dvec3 position;
};

template <typename Objects>
class KdTree {
    public:
        KdTree(BoundingBox box) : bbox(box) {}
        KdTree buildTree(std::vector<Objects *> objList, BoundingBox bbox, int depth, int leafSize);
        Plane findBestSplitPlane(std::vector<Objects *> objList, BoundingBox bbox);
        int countObjects(std::vector<Objects *> objList, BoundingBox bbox);

        bool findIntersection(ray &r, isect &i, double tmin, double tmax);

        BoundingBox getBoundingBox() {return bbox;}

    private:
        // KdTree *left;
        // KdTree *right;
        BoundingBox bbox;

};



template<typename Objects>
class SplitNode : KdTree<Objects> {
        Plane plane;
        KdTree<Objects> *left;
        KdTree<Objects> *right;
    public:
        
        SplitNode(Plane pl, KdTree<Objects> l, KdTree<Objects> r, BoundingBox box) : plane(pl), left(&l), right(&r), KdTree<Objects>(box) {}
        bool findIntersection(ray &r, isect &i, double tmin, double tmax);
    private:
        

    
};

template<typename Objects>
class LeafNode : KdTree<Objects> {
    public:
        LeafNode(std::vector<Objects *> o, BoundingBox box) : objList(o), BoundingBox(box) {}
        bool findIntersection(ray &r, isect &i, double tmin, double tmax);
    private:
        std::vector<Objects *> objList;
};