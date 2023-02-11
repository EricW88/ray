#include "kdTree.h"

KdTree<Objects>* KdTree<Objects>::buildTree(std::vector<Objects*> objList, BoundingBox bbox, int depth, int leafSize) {
    if(objList.size() <= leafSize || depth == 0) {
        return &LeafNode(objList, bbox);
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
        }
        // account for objects on the left and right of the split
        if(glm::dot(bboxMax > position) {
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
        return &LeafNode(objList, bbox);
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
    return &SplitNode(bestPlane, buildTree(leftList, leftBBox, depth - 1, leafSize), buildTree(rightList, rightBBox, depth - 1, leafSize), bbox);
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

        if(candidate.getAxis()[0] == 1) {
            leftMax[0] = candidate.getPosition()[0];
            rightMin[0] = candidate.getPosition()[0];
        } else if (candidate.getAxis()[1] == 1) {
            leftMax[1] = candidate.getPosition()[1];
            rightMin[1] = candidate.getPosition()[1];
        } else {
            leftMax[2] = candidate.getPosition()[2];
            rightMin[2] = candidate.getPosition()[2];
        }
        

        BoundingBox leftBox(bbox.getMin(), leftMax);
        BoundingBox rightBox(rightMin, bbox.getMax());


        int leftCount = 0;
        int rightCount = 0;


        for(Objects *obj : objList) {
            BoundingBox objBox = obj->getBoundingBox();
            double position = glm::dot(candidate.getPosition(), candidate.getAxis());
            double bboxMin = glm::dot(objBox.bmin, candidate.getAxis());
            double bboxMax = glm::dot(objBox.bmax, candidate.getAxis());
            if(bboxMin < position) {
                leftCount++;
            }
            // account for objects on the left and right of the split
            if(glm::dot(bboxMax > position) {
                rightCount++;
            }
        }

        double sam = leftCount * leftBox.area() + rightCount * rightBox.area();
        if(sam < minSam) {
            minSam = sam;
            bestPlane = plane;
        }
    }

    return bestPlane;
}

// int kdTree::countObjects(std::vector<Objects *> objList, BoundingBox bbox) {
//     int result = 0;
//     for(Objects *obj : objList) {
//         BoundingBox objBox = obj->getBoundingBox();
//         if(bbox.intersects(objBox)) ++result;
//     }
//     return result;
// }


bool SplitNode::findIntersection(ray &r, isect &i, double tmin, double tmax, bool &found_one) {
    // don't check for intersection in left and right bbox individually b/c it's slow
    bool hitBbox = this->getBoundingBox().intersect(r, tmin, tmax);
    // didn't intersect the bounding box
    if(!hitBBox) return false;

    double r_pos = glm::dot(r.getPosition(), plane.getAxis());
    double r_dir = glm::dot(r.getDirection(), plane.getAxis());
    double axis_pos = glm::dot(plane.getPosition(), plane.getAxis());

    // ray is parallel
    if(r_dir == 0) {
        if(r_pos < axis_pos && left->findIntersection(r, i, tmin, tmax, found_one)) {
            return true;
        } else if(r_pos > axis_pos && right->findIntersection(r, i, tmin, tmax, found_one)) {
            return true;
        }    
    }
    double t_prime = (axis_pos - r_pos) / r_dir;

    if(t_prime < 0) return false;

    // check if hit both boxes
    if((tmin < t_prime && t_prime < tmax)) {
        bool leftRecurse = left->findIntersection(r, i, tmin, tmax, found_one);
        bool rightRecurse = right->findIntersection(r, i, tmin, tmax, found_one);
        return leftRecurse || rightRecurse;
    }

    else if(glm::dot(ray.at(tmin), plane.getAxis()) < axis_pos && left->findIntersection(r, i, tmin, tmax, found_one)) {
            return true;
    }
    else if(glm::dot(ray.at(tmin), plane.getAxis()) > axis_pos && right->findIntersection(r, i, tmin, tmax, found_one)) {
            return true;
    }
    return false;
    
}

bool LeafNode::findIntersection(ray &r, isect &i, double tmin, double tmax, bool &found_one) {
    bool result = false;
    for(Objects *obj : objList) {
        isect c_i;
        if(obj->intersect(r, c_i) && c_i.getT() >= tmin && c_i.getT() <= tmax) {
            result = true;
            if(!found_one || c_i.getT() < i.getT()) {
                found_one = true;
                i = c_i;
            }
        }
    }
    return result;
}