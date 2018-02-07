//
//  Point.hpp
//  HausdorffDistance
//
//  Created by 李喆 on 2018/1/3.
//  Copyright © 2018年 李喆. All rights reserved.
//


#ifndef Point_hpp
#define Point_hpp

#include <stdio.h>
#include <math.h>
#include <limits>

class Point{
public:
    Point(double x = 0, double y = 0, double z = 0, int index = 0, int dimension = 3, bool isAvailable = true, bool isCenter = false, uint64_t zorder = 0);
    double x;
    double y;
    double z;
    int index;
    int dimension;
    bool isAvailable;
    bool isCenter;
    uint64_t zorder;
    
    double distanceTo(const Point &point);
    double getX() const {return x;};
    void setX(double x){this->x = x;};
    double getY() const {return y;};
    void setY(double y){this->y = y;};
    double getZ() const {return z;};
    void setZ(double z){this->z = z;};
};

Point::Point(double x, double y, double z, int index, int dimension, bool isAvailable, bool isCenter, uint64_t zorder){
    this->x = x;
    this->y = y;
    this->z = z;
    this->index = index;
    this->dimension = dimension;
    this->isAvailable = isAvailable;
    this->isCenter = isCenter;
    this->zorder = zorder;
}

double Point::distanceTo(const Point &point){
    double distance = 0;
    if(dimension == 2 && point.dimension == 2){
        double distanceX = fabs(this->x - point.x);
        double distanceY = fabs(this->y - point.y);
        distance = sqrt(distanceX*distanceX + distanceY*distanceY);
    } else if (dimension == 3 && point.dimension == 3){
        double distanceX = fabs(this->x - point.x);
        double distanceY = fabs(this->y - point.y);
        double distanceZ = fabs(this->z - point.z);
        distance = sqrt(distanceX*distanceX + distanceY*distanceY + distanceZ*distanceZ);
    }
    return distance;
}
#endif /* Point_hpp */
