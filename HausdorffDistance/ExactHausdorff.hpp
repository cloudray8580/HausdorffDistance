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
#include "Dataset.hpp"
#include <limits>

class ExactHausdorff{
public:
    static double definition(PointCloud pc1, PointCloud pc2);
    static double PAMI2015(PointCloud &pc1, PointCloud &pc2, bool pruning=false, double kthValue=std::numeric_limits<double>::infinity()); // without excluding intersection
    static double PR2017(PointCloud &pc1, PointCloud &pc2);
    
    static double PAMI2015_recordMax(PointCloud &pc1, PointCloud &pc2, string filepath, bool pruning=false, double kthValue=std::numeric_limits<double>::infinity());
    static double Partial_PAMI2015(PointCloud &pc1, PointCloud &pc2, int progress);
    static double Partial_PAMI205_Pruning(PointCloud &pc1, PointCloud &pc2, int start, int end, double currentMax, double currentKNN);
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

double ExactHausdorff::PAMI2015(PointCloud &pc1, PointCloud &pc2, bool pruning, double kthValue){
    double max = 0;
    double min = std::numeric_limits<double>::infinity(); // infinity
    double distance = 0;
    
    // seperate to count time
//    pc1.randomize();
//    pc2.randomize();
//    vector<Point> pointcloud1 = pc1.getPoints_1();
//    vector<Point> pointcloud2 = pc2.getPoints_1();
    vector<Point> *pointcloud1 = &pc1.pointcloud;
    vector<Point> *pointcloud2 = &pc2.pointcloud;
    
    long size1 = pointcloud1->size();
    long size2 = pointcloud2->size();
    for (int i = 0; i < size1; i++){
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; j < size2; j++){
            distance = (*pointcloud1)[i].distanceTo((*pointcloud2)[j]);
            if(distance < min){
                min = distance;
            }
            if(distance <= max){
                break;
            }
        }
        if(min > max){
            max = min;
            if(pruning && max >= kthValue){
                //max = -1; // usig -1 to denote early break
                return max;
            }
        }
        
    }
    return max; // the Hausdorff distance
}

double ExactHausdorff::PR2017(PointCloud &pc1, PointCloud &pc2){
    double max = 0;
    double min = std::numeric_limits<double>::infinity(); // infinity
    double distance = 0;
    // seperate to count time
//    pc1.zorder();
//    pc2.zorder();
//    vector<Point> pointcloud1 = pc1.getPoints_1();
//    vector<Point> pointcloud2 = pc2.getPoints_1();
    vector<Point> *pointcloud1 = &pc1.pointcloud;
    vector<Point> *pointcloud2 = &pc2.pointcloud;
    
    long size1 = pointcloud1->size();
    long size2 = pointcloud2->size();
    int breakindex = 0;
    for (int i = 0; i < size1; i++){
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; breakindex+j < size2 || breakindex-j > 0 ; j++){
            if (breakindex+j < size2){
                distance = (*pointcloud1)[i].distanceTo((*pointcloud2)[breakindex+j]);
                if(distance < min){
                    min = distance;
                }
                if(distance < max){
                    breakindex = breakindex+j;
                    break;
                }
            }
            if (breakindex-j > 0){
                distance = (*pointcloud1)[i].distanceTo((*pointcloud2)[breakindex-j]);
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

double ExactHausdorff::PAMI2015_recordMax(PointCloud &pc1, PointCloud &pc2, string filepath, bool pruning, double kthValue){
    double max = 0;
    double min = std::numeric_limits<double>::infinity(); // infinity
    double distance = 0;
    
    // seperate to count time
    pc1.randomize();
    pc2.randomize();
    //    vector<Point> pointcloud1 = pc1.getPoints_1();
    //    vector<Point> pointcloud2 = pc2.getPoints_1();
    vector<Point> *pointcloud1 = &pc1.pointcloud;
    vector<Point> *pointcloud2 = &pc2.pointcloud;
    
    ofstream outfile;
    outfile.open(filepath);
    
    long size1 = pointcloud1->size();
    long size2 = pointcloud2->size();
    for (int i = 0; i < size1; i++){
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; j < size2; j++){
            distance = (*pointcloud1)[i].distanceTo((*pointcloud2)[j]);
            if(distance < min){
                min = distance;
            }
            if(distance <= max){
                break;
            }
        }
        if(min > max){
            max = min;
            if(pruning && max >= kthValue){
                //max = -1; // usig -1 to denote early break
                return max;
            }
        }
        outfile << i << " " << max << endl;
    }
    return max; // the Hausdorff distance
}

double ExactHausdorff::Partial_PAMI2015(PointCloud &pc1, PointCloud &pc2, int progress){
    double max = 0;
    double min = std::numeric_limits<double>::infinity();
    double distance = 0;
    if(progress > pc1.pointcloud.size()){
        progress = pc1.pointcloud.size();
    }
    for (int i = 0; i < progress; i++){
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; j < pc2.pointcloud.size(); j++){
            distance = pc1.pointcloud[i].distanceTo(pc2.pointcloud[j]);
            if(distance < min){
                min = distance;
            }
            if(distance <= max){
                break;
            }
        }
        if(min > max){
            max = min;
        }
    }
    return max;
}

// if break by threshold, the max value is -1
inline double ExactHausdorff::Partial_PAMI205_Pruning(PointCloud &pc1, PointCloud &pc2, int start, int end, double currentMax, double currentKNN){
    double max = currentMax;
    double min = std::numeric_limits<double>::infinity();
    double distance = 0;
    if(start >= end)
        return currentMax;

    for (int i = start; i < end; i++){
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; j < pc2.pointcloud.size(); j++){
            distance = pc1.pointcloud[i].distanceTo(pc2.pointcloud[j]);
            if(distance < min){
                min = distance;
            }
            if(distance <= max){
                break;
            }
        }
        if(min > max){
            max = min;
            if(max > currentKNN){
                return -1; // should we change to max = -1 to denote early break? since this distance will not be used anymore
            }
        }
    }
    return max;
}

#endif /* ExactHausdorff_hpp */
