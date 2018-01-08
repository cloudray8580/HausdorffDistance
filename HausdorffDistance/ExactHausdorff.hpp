//
//  ExactHausdorff.hpp
//  HausdorffDistance
//
//  Created by 李喆 on 2018/1/3.
//  Copyright © 2018年 李喆. All rights reserved.
//

#ifndef ExactHausdorff_hpp
#define ExactHausdorff_hpp

#include <stdio.h>
#include "PointCloud.hpp"
#include <limits>
#endif /* ExactHausdorff_hpp */

class ExactHausdorff{
public:
    static double definition(PointCloud pc1, PointCloud pc2);
    static double PAMI2015(PointCloud pc1, PointCloud pc2); // without excluding intersection
    static double PR2017(PointCloud pc1, PointCloud pc2);
};

double ExactHausdorff::definition(PointCloud pc1, PointCloud pc2){
    double max = 0;
    double min = std::numeric_limits<double>::infinity(); // infinity
    double distance = 0;
    vector<Point> pointcloud1 = pc1.getPoints_1();
    vector<Point> pointcloud2 = pc2.getPoints_1();
    long size1 = pointcloud1.size();
    long size2 = pointcloud2.size();
    for (int i = 0; i < size1; i++){
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; j < size2; j++){
            distance = pointcloud1[i].distanceTo(pointcloud2[j]);
            if(distance < min){
                min = distance;
            }
        }
        if(min > max){
            max = min;
        }
    }
    return max; // the Hausdorff distance
}

double ExactHausdorff::PAMI2015(PointCloud pc1, PointCloud pc2){
    double max = 0;
    double min = std::numeric_limits<double>::infinity(); // infinity
    double distance = 0;
    pc1.randomize();
    pc2.randomize();
    vector<Point> pointcloud1 = pc1.getPoints_1();
    vector<Point> pointcloud2 = pc2.getPoints_1();
    long size1 = pointcloud1.size();
    long size2 = pointcloud2.size();
    for (int i = 0; i < size1; i++){
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; j < size2; j++){
            distance = pointcloud1[i].distanceTo(pointcloud2[j]);
            if(distance < min){
                min = distance;
            }
            if(distance < max){
                break;
            }
        }
        if(min > max){
            max = min;
        }
    }
    return max; // the Hausdorff distance
}

double ExactHausdorff::PR2017(PointCloud pc1, PointCloud pc2){
    double max = 0;
    double min = std::numeric_limits<double>::infinity(); // infinity
    double distance = 0;
    pc1.zorder();
    pc2.zorder();
    vector<Point> pointcloud1 = pc1.getPoints_1();
    vector<Point> pointcloud2 = pc2.getPoints_1();
    long size1 = pointcloud1.size();
    long size2 = pointcloud2.size();
    int breakindex = 0;
    for (int i = 0; i < size1; i++){
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; breakindex+j < size2 || breakindex-j > 0 ; j++){
            if (breakindex+j < size2){
                distance = pointcloud1[i].distanceTo(pointcloud2[breakindex+j]);
                if(distance < min){
                    min = distance;
                }
                if(distance < max){
                    breakindex = breakindex+j;
                    break;
                }
            }
            if (breakindex-j > 0){
                distance = pointcloud1[i].distanceTo(pointcloud2[breakindex-j]);
                if(distance < min){
                    min = distance;
                }
                if(distance < max){
                    breakindex = breakindex-j;
                    break;
                }
            }
        }
        if(min > max){
            max = min;
        }
    }
    return max; // the Hausdorff distance
}
