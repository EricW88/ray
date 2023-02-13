#include "kdTree.h"
#include <iostream>

bool Plane::operator < (const Plane& other) const {
    return glm::dot(axis, position) < glm::dot(other.axis, other.position);
}

template <typename Objects>
KdTree<Objects>* KdTree<Objects>::buildTree(std::vector<Objects*> objList, BoundingBox bbox, int depth, int leafSize) {
    // std::cout << "recursing..." << objList.size() << std::endl;
    if(objList.size() <= leafSize || depth == 0) {
        return new LeafNode<Objects>(objList, bbox);
    }
    Plane bestPlane = findBestSplitPlane(objList, bbox);
    std::vector<Objects *> leftList;
    std::vector<Objects *> rightList;
    for(Objects *obj : objList) {
        bool enteredList = false;
        BoundingBox objBox = obj->getBoundingBox();
        double position = glm::dot(bestPlane.getPosition(), bestPlane.getAxis());
        double bboxMin = glm::dot(objBox.getMin(), bestPlane.getAxis());
        double bboxMax = glm::dot(objBox.getMax(), bestPlane.getAxis());
        if(bboxMin < position) {
            enteredList = true;
            leftList.push_back(obj);
        }
        // account for objects on the left and right of the split
        if(bboxMax > position) {
            enteredList = true;
            rightList.push_back(obj);
        }

        double objNormal = glm::dot(obj->getNormal(), bestPlane.getAxis());
        if(bboxMax == position && bboxMin == position && objNormal < 0) {
            enteredList = true;
            leftList.push_back(obj);
        } else if(bboxMax == position && bboxMin == position && objNormal >= 0) {
            enteredList = true;
            rightList.push_back(obj);
        }
    }

    if(rightList.empty() || leftList.empty()) {
        return new LeafNode<Objects>(objList, bbox);
    }
    // std::cout << leftList.size() << " " << rightList.size() << std::endl;
    glm::dvec3 leftMax = bbox.getMax();
    glm::dvec3 rightMin = bbox.getMin();
    if(bestPlane.getAxis()[0] != 0) {
        leftMax[0] = bestPlane.getPosition()[0];
        rightMin[0] = bestPlane.getPosition()[0];
    } else if(bestPlane.getAxis()[1] != 0) {
        leftMax[1] = bestPlane.getPosition()[1];
        rightMin[1] = bestPlane.getPosition()[1];
    } else {
        leftMax[2] = bestPlane.getPosition()[2];
        rightMin[2] = bestPlane.getPosition()[2];
    }
    BoundingBox leftBBox = BoundingBox(bbox.getMin(), leftMax);
    BoundingBox rightBBox = BoundingBox(rightMin, bbox.getMax());
    return new SplitNode<Objects>(bestPlane, buildTree(leftList, leftBBox, depth - 1, leafSize), buildTree(rightList, rightBBox, depth - 1, leafSize), bbox);
}

// template <typename Objects>
// KdTree<Objects>::KdTree(std::vector<Objects *> objList, BoundingBox box, int depth, int leafSize) : bbox(box){
//     this = buildTree(objList, bbox, depth, leafSize);
// }

template <typename Objects>
Plane KdTree<Objects>::findBestSplitPlane(std::vector<Objects *> objList, BoundingBox bbox) {
    double minSam = DBL_MAX;
    Plane bestPlane;

    for(int a = 0; a < 3; ++a) {
        std::vector<Plane> candidateList;
        glm::dvec3 axis(0, 0, 0);
        axis[a] = 1;
        double bboxMin = glm::dot(axis, bbox.getMin());
        double bboxMax = glm::dot(axis, bbox.getMax());
        for(Objects *obj : objList) {
            BoundingBox objBox = obj->getBoundingBox();


            Plane minPlane(axis, objBox.getMin(), true);
            Plane maxPlane(axis, objBox.getMax(), false);
            candidateList.push_back(minPlane);
            candidateList.push_back(maxPlane);
        }

        std::sort(candidateList.begin(), candidateList.end());
        int leftCount = 0;
        int rightCount = objList.size();
        for(Plane candidate : candidateList) {
            if(!candidate.isLeft()) {
                ++leftCount;
                --rightCount;
            }

            glm::dvec3 leftMax = bbox.getMax();
            glm::dvec3 rightMin = bbox.getMin();
            leftMax[a] = candidate.getPosition()[a];
            rightMin[a] = candidate.getPosition()[a];
            BoundingBox leftBox(bbox.getMin(), leftMax);
            BoundingBox rightBox(rightMin, bbox.getMax());
            double sam = leftCount * leftBox.area() + rightCount * rightBox.area();
            if(sam < minSam) {
                minSam = sam;
                bestPlane = candidate;
            }
        }
    }
    // std::cout << "Returning!!!" << std::endl;
    return bestPlane;
}


template <typename Objects>
void SplitNode<Objects>::findIntersection(ray &r, isect &i, double tmin, double tmax, bool &found_one) {

    bool hitBbox = this->getBoundingBox().intersect(r, tmin, tmax);
    // didn't intersect the bounding box
    if(!hitBbox) return;


    double r_pos = glm::dot(r.getPosition(), plane.getAxis()); // starting position of ray
    double r_dir = glm::dot(r.getDirection(), plane.getAxis()); // ray direction
    double axis_pos = glm::dot(plane.getPosition(), plane.getAxis()); // point on axis


    if(r_dir == 0) {
        if(r_pos <= axis_pos) {
            this->left->findIntersection(r, i, tmin, tmax, found_one);
        } 
        if(r_pos >= axis_pos) {
            this->right->findIntersection(r, i, tmin, tmax, found_one);
        }  
        return;
    }

    double t_prime = (axis_pos - r_pos) / r_dir;

    if(t_prime < 0) {
        if(glm::dot(r.at(tmax), plane.getAxis()) <= axis_pos) {
            // assert(glm::dot(r.at(tmax), plane.getAxis()) <= axis_pos);
            this->left->findIntersection(r, i, tmin, tmax, found_one);
        }
        if(glm::dot(r.at(tmax), plane.getAxis()) >= axis_pos) {
            // assert(glm::dot(r.at(tmax), plane.getAxis()) >= axis_pos);
            this->right->findIntersection(r, i, tmin, tmax, found_one);
        }
        return;
    }

    if((tmin <= t_prime && t_prime <= tmax)) {
        this->left->findIntersection(r, i, tmin, tmax, found_one);
        this->right->findIntersection(r, i, tmin, tmax, found_one);
    } else {
        
        if(glm::dot(r.at(tmin), plane.getAxis()) <= axis_pos) {
            // assert(glm::dot(r.at(tmax), plane.getAxis()) <= axis_pos);
            this->left->findIntersection(r, i, tmin, tmax, found_one);
        }
        if(glm::dot(r.at(tmin), plane.getAxis()) >= axis_pos) {
            // assert(glm::dot(r.at(tmax), plane.getAxis()) >= axis_pos);
            this->right->findIntersection(r, i, tmin, tmax, found_one);
        }
    }
    
    
    
}

template <typename Objects>
void LeafNode<Objects>::findIntersection(ray &r, isect &i, double tmin, double tmax, bool &found_one) {
    bool hitBbox = this->getBoundingBox().intersect(r, tmin, tmax);
    // didn't intersect the bounding box
    if(!hitBbox) return;
    // bool result = false;
    for(Objects *obj : objList) {
        isect c_i;
        if(obj->intersect(r, c_i) && c_i.getT() >= tmin && c_i.getT() <= tmax) {
            // result = true;
            if(!found_one || c_i.getT() < i.getT()) {
                found_one = true;
                i = c_i;
            }
        }
    }
    // return result;
}