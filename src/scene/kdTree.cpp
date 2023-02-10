#include "kdTree.h"

kdTree kdTree::buildTree(std::vector<Objects *> objList, BoundingBox bbox, int depth, int leafSize) {
    if(objList.size() <= leafSize || depth == 0) {
        return LeafNode(objList, bbox);
    }
    Plane bestPlane = findBestSplitPlane(objList, bbox);
    std::vector<Objects *> leftList;
    std::vector<Objects *> rightList;
    for(Objects *obj : objList) {
        BoundingBox objBox = obj->getBoundingBox();
        double position = glm::dot(bestPlane.position, bestPlane.axis);
        double bboxMin = glm::dot(objBox.bmin, bestPlane.axis);
        double bboxMax = glm::dot(objBox.bmax, bestPlane.axis);
        if(bboxMin < position) {
            leftList.push_back(obj);
        } else if(glm::dot(bboxMax > position) {
            rightList.push_back(obj);
        }

        double objNormal = glm::dot(obj->getNormal(), bestPlane.axis);
        if(bboxMax == position && bboxMin == position && objNormal < 0) {
            leftList.push_back(obj);
        } else if(bboxMax == position && bboxMin == position && objNormal >= 0) {
            rightList.push_back(obj);
        }
    }

    if(rightList.empty() || leftList.empty()) {
        return LeafNode(objList, bbox);
    }

    glm::dvec3 leftMax = bbox.getMax();
    glm::dvec3 rightMin = bbox.getMin();
    if(bestPlane.axis.x != 0) {
        leftMax.x = bestPlane.position.x;
        rightMin.x = bestPlane.position.x;
    } else if(bestPlane.axis.y != 0) {
        leftMax.y = bestPlane.position.y;
        rightMin.y = bestPlane.position.y;
    } else {
        leftMax.z = bestPlane.position.z;
        rightMin.z = bestPlane.position.z;
    }
    BoundingBox leftBBox = BoundingBox(bbox.getMin(), leftMax);
    BoundingBox rightBBox = BoundingBox(rightMin, bbox.getMax());
    return SplitNode(bestPlane, buildTree(leftList, leftBBox, depth - 1, leafSize), buildTree(rightList, rightBBox, depth - 1, leafSize), bbox);
}

Plane kdTree::findBestSplitPlane(std::vector<Objects *> objList, BoundingBox bbox) {
    std::vector<Plane> candidateList;
    for(int a = 0; a < 3; ++a) {
        glm::dvec3 axis(0, 0, 0);
        axis[a] = 1;
        for(Objects *obj : objList) {
            BoundingBox objBox = obj->getBoundingBox();
            Plane minPlane(axis, objBox.getMin());
            Plane maxPlane(axis, objBox.getMax());
            candidateList.push_back(minPlane);
            candidateList.push_back(maxPlane);
        }
    }

    double minSam = DBL_MAX;
    Plane bestPlane;
    for(Plane candidate : candidateList) {
        glm::dvec3 leftMax = bbox.getMax();
        glm::dvec3 rightMin = bbox.getMin();
        leftMax[a] = candidate.position[a];
        rightMin[a] = candidate.position[a];

        BoundingBox leftBox(bbox.getMin(), leftMax);
        BoundingBox rightBox(rightMin, bbox.getMax());
        int leftCount = countObjects(objList, leftBox);
        int rightCount = countObjects(objList, rightBox);

        double sam = leftCount * leftBox.area() + rightCount * rightBox.area();
        if(sam < minSam) {
            minSam = sam;
            bestPlane = plane;
        }
    }

    return bestPlane;
}

int kdTree::countObjects(std::vector<Objects *> objList, BoundingBox bbox) {
    int result = 0;
    for(Objects *obj : objList) {
        BoundingBox objBox = obj->getBoundingBox();
        if(bbox.intersects(objBox)) ++result;
    }
    return result;
}


bool SplitNode::findIntersection(ray &r, isect &i, double tmin, double tmax) {
    bool hitLeft = left->getBoundingBox().intersect(r, tmin, tmax);
    bool hitRight = right->getBoundingBox().intersect(r, tmin, tmax);

    if(hitLeft && hitRight) {
        bool leftRecurse = left->findIntersection(r, i, tmin, tmax);
        bool rightRecurse = right->findIntersection(r, i, tmin, tmax);
        return leftRecurse || rightRecurse;
    }

    else if(hitLeft && left->findIntersection(r, i, tmin, tmax)) {
            return true;
    }
    else if(hitRight && right->findIntersection(r, i, tmin, tmax)) {
            return true;
    }
    return false;
    
}

bool LeafNode::findIntersection(ray &r, isect &i, double tmin, double tmax) {
    bool result = false;
    for(Objects *obj : objList) {
        isect c_i;
        if(obj.intersect(r, c_i) && c_i.getT() >= tmin && c_i.getT() <= tmax) {
            result = true;
            if(c_i.getT() < i.getT()) {
                i = c_i;
            }
        }
    }
    return result;
}