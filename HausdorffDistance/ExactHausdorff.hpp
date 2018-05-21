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
    
    static double PAMI2015_UsingHilbert(PointCloud &pc1, PointCloud &pc2, bool pruning=false, double kthValue=std::numeric_limits<double>::infinity());
    
    // call sortByKCenter first!!!
    static double PAMI2015_Pruning_KCenterUB(PointCloud &pc1, PointCloud &pc2, vector<pair<double,int>> &disToKcenter, double kthValue=std::numeric_limits<double>::infinity());
    
    static double Partial_PAMI2015(PointCloud &pc1, PointCloud &pc2, int progress);
    static double Partial_PAMI2015(PointCloud &pc1, PointCloud &pc2, int start, int end, double currentMax);
    static double Partial_PAMI2015_Pruning(PointCloud &pc1, PointCloud &pc2, int start, int end, double currentMax, double currentKNN);
    
    struct maxAndDistances{
        double max;
        vector<double> distances;
        maxAndDistances(double m, vector<double> &d):max(m), distances(d){}
    };
    
    static double Partial_PAMI2015_Pruning_KCenterUB(PointCloud &pc1, PointCloud &pc2, int start, int end, double currentMax, double currentKNN, vector<pair<double,int>> &disToKcenter, ExactHausdorff::maxAndDistances &result);
    
    static maxAndDistances Partial_PAMI2015_Pruning_WithRecord(PointCloud &pc1, PointCloud &pc2, int start, int end, double currentMax, double currentKNN);
    static maxAndDistances Partial_PAMI2015_Pruning_WithRecord_FullInnerloop(PointCloud &pc1, PointCloud &pc2, int start, int end, double currentMax, double currentKNN);
    
    static double PAMI2015_Log(PointCloud &pc1, PointCloud &pc2, bool pruning=false, double kthValue=std::numeric_limits<double>::infinity());
    
    static double LowerboundFromKCenterToBound(PointCloud &pc, pair<Point, Point> &bound, double kthValue);
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
                max = -1; // usig -1 to denote early break
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

inline int binarySearch(double value, int start, int end, PointCloud& pc){
    
    if ((end - start <= 1) || start >= pc.pointcloud.size() || end == 0) {
        return end;
    }
    
    int middle = (end+start)/2;
    double middleValue = pc.pointcloud[middle].hilbertValue;
    if (fabs(middleValue - value) < 0.000001){
        return middle;
    } else if (middleValue < value) {
        return binarySearch(value, middle, end, pc);
    } else {
        return binarySearch(value, start, middle, pc);
    }
}

inline int binarySearchNonRecursive(double value, int start, int end, PointCloud& pc){
    
    if ((end - start <= 1) || start >= pc.pointcloud.size() || end == 0) {
        return end;
    }
    int middle = 0;
    while(end > start){
        middle = (end+start)/2;
        double middleValue = pc.pointcloud[middle].hilbertValue;
        if (fabs(middleValue - value) < 0.000001){
            return middle;
        } else if (middleValue < value) {
            start = middle+1;
        } else {
            end = middle-1;
        }
    }
    return middle;
}

double ExactHausdorff::PAMI2015_UsingHilbert(PointCloud &pc1, PointCloud &pc2, bool pruning, double kthValue){
    double max = 0;
    double min = std::numeric_limits<double>::infinity(); // infinity
    double distance = 0;

    vector<Point> *pointcloud1 = &pc1.pointcloud;
    vector<Point> *pointcloud2 = &pc2.pointcloud;
    
//    vector<Point>& pointcloud1 = pc1.pointcloud;
//    vector<Point>& pointcloud2 = pc2.pointcloud;
    
    long size1 = pointcloud1->size();
    long size2 = pointcloud2->size();
//    long size1 = pointcloud1.size();
//    long size2 = pointcloud2.size();
    double HilbertUB = std::numeric_limits<double>::infinity();
    
    for (int i = 0; i < size1; i++){
        
//        int startPosition = binarySearch(pc1.pointcloud[i].hilbertValue, 0, size2-1, pc2);
        int startPosition = binarySearchNonRecursive(pc1.pointcloud[i].hilbertValue, 0, size2-1, pc2);
        min = std::numeric_limits<double>::infinity();
        HilbertUB = pc1.pointcloud[i].distanceTo(pc2.pointcloud[startPosition]);
//        HilbertUB = pointcloud1[i].distanceTo(pointcloud2[startPosition]);
        
        // ignore this inner loop
        if (HilbertUB <= max){
            continue;
        } else {
            min = HilbertUB;
        }
        
        for(int j = startPosition, step = 1; j-step > 0 || j+step < size2 ; step++){
            if (j-step > 0){
                distance = (*pointcloud1)[i].distanceTo((*pointcloud2)[j-step]);
//                distance = pointcloud1[i].distanceTo(pointcloud2[j-step]);
                if(distance < min){
                    min = distance;
                }
                if(distance <= max){
                    break;
                }
            }
            if (j+step < size2){
                distance = (*pointcloud1)[i].distanceTo((*pointcloud2)[j+step]);
//                distance = pointcloud1[i].distanceTo(pointcloud2[j+step]);
                if(distance < min){
                    min = distance;
                }
                if(distance <= max){
                    break;
                }
            }
        }
        
        if(min > max){
            max = min;
            if(pruning && max >= kthValue){
                max = -1; // usig -1 to denote early break
                return max;
            }
        }
        
    }
    return max; // the Hausdorff distance
}

// call sortByKCenter first!!!
double ExactHausdorff::PAMI2015_Pruning_KCenterUB(PointCloud &pc1, PointCloud &pc2, vector<pair<double,int>> &disToKcenter, double kthValue){
    
    double max = 0;
    double min = std::numeric_limits<double>::infinity();
    double distance = 0;
    int kcenterIndex = 0;
    
    int KNum = pc1.getKCenterNum();
    double distToData[KNum];
//    int breakcount = 0;
    
    for (int i = 0; i < pc1.pointcloud.size(); i++){
        
        if(i >= KNum){
            kcenterIndex = disToKcenter[i-KNum].second;
            distance = disToKcenter[i-KNum].first + distToData[kcenterIndex];
            if(distance < max){
//                breakcount++;
                continue;
            }
        }
        
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; j < pc2.pointcloud.size(); j++){
            distance = pc1.pointcloud[i].distanceTo(pc2.pointcloud[j]);
            if(distance < min){
                min = distance;
            }
            if(distance < max){
                break;
            }
        }
        if(min > max){
            max = min;
            if(max > kthValue){
                return -1;
            }
        }
        
        if(i < KNum)
            distToData[i] = distance;
    }
//    cout << "breakcount : " << breakcount << endl;
    return max;
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
            if(distance < max){
                break;
            }
        }
        if(min > max){
            max = min;
        }
    }
    return max;
}

double ExactHausdorff::Partial_PAMI2015(PointCloud &pc1, PointCloud &pc2, int start, int end, double currentMax){
    
    double max = currentMax;
    double min = std::numeric_limits<double>::infinity();
    double distance = 0;
    
    if(end > pc1.pointcloud.size())
        end = pc1.pointcloud.size();
    
    if(start >= end)
        return currentMax;
    
    for (int i = start; i < end; i++){
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; j < pc2.pointcloud.size(); j++){
            distance = pc1.pointcloud[i].distanceTo(pc2.pointcloud[j]);
            if(distance < min){
                min = distance;
            }
            if(distance <= max){ // if =, if migth break in the first outer loop
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
inline double ExactHausdorff::Partial_PAMI2015_Pruning(PointCloud &pc1, PointCloud &pc2, int start, int end, double currentMax, double currentKNN){
    
    double max = currentMax;
    double min = std::numeric_limits<double>::infinity();
    double distance = 0;
    
    if(end > pc1.pointcloud.size())
        end = pc1.pointcloud.size();
    
    if(start >= end)
        return currentMax;

    for (int i = start; i < end; i++){
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; j < pc2.pointcloud.size(); j++){
            distance = pc1.pointcloud[i].distanceTo(pc2.pointcloud[j]);
            if(distance < min){
                min = distance;
            }
            if(distance < max){
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

inline double ExactHausdorff::Partial_PAMI2015_Pruning_KCenterUB(PointCloud &pc1, PointCloud &pc2, int start, int end, double currentMax, double currentKNN, vector<pair<double,int>> &disToKcenter, ExactHausdorff::maxAndDistances &result){
    double max = currentMax;
    double min = std::numeric_limits<double>::infinity();
    double distance = 0;
    
    if(end > pc1.pointcloud.size())
        end = pc1.pointcloud.size();
    
    if(start >= end)
        return currentMax;
    
    int kcenterIndex = 0;
//    int breakByKCUB = 0;
    
    for (int i = start,j=0; i < end; i++,j++){
        
        kcenterIndex = disToKcenter[j].second;
        // UB still statisfy
        distance = disToKcenter[j].first + result.distances[kcenterIndex];
        if(distance < max){
//            breakByKCUB++;
            continue;
        }
        
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; j < pc2.pointcloud.size(); j++){
            distance = pc1.pointcloud[i].distanceTo(pc2.pointcloud[j]);
            if(distance < min){
                min = distance;
            }
            if(distance < max){
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
//    cout << "break by kcenterUB: " << breakByKCUB << endl;
    return max;
}

ExactHausdorff::maxAndDistances ExactHausdorff::Partial_PAMI2015_Pruning_WithRecord(PointCloud &pc1, PointCloud &pc2, int start, int end, double currentMax, double currentKNN){
    double max = currentMax;
    double min = std::numeric_limits<double>::infinity();
    double distance = 0;
    vector<double> distances;
    
    if(end > pc1.pointcloud.size())
        end = pc1.pointcloud.size();
    
    if(start >= end){
        return maxAndDistances(currentMax, distances);
    }
    
    for (int i = start; i < end; i++){
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; j < pc2.pointcloud.size(); j++){
            distance = pc1.pointcloud[i].distanceTo(pc2.pointcloud[j]);
            if(distance < min){
                min = distance;
            }
            if(distance < max){
                break;
            }
        }
        distances.push_back(min);
        if(min > max){
            max = min;
            if(max > currentKNN){
                max = -1; // should we change to max = -1 to denote early break? since this distance will not be used anymore
                break;
            }
        }
    }
    
    return maxAndDistances(max, distances);
}

ExactHausdorff::maxAndDistances ExactHausdorff::Partial_PAMI2015_Pruning_WithRecord_FullInnerloop(PointCloud &pc1, PointCloud &pc2, int start, int end, double currentMax, double currentKNN){
    double max = currentMax;
    double min = std::numeric_limits<double>::infinity();
    double distance = 0;
    vector<double> distances;
    
    if(end > pc1.pointcloud.size())
        end = pc1.pointcloud.size();
    
    if(start >= end){
        return maxAndDistances(currentMax, distances);
    }
    
    for (int i = start; i < end; i++){
        min = std::numeric_limits<double>::infinity();
        for(int j = 0; j < pc2.pointcloud.size(); j++){
            distance = pc1.pointcloud[i].distanceTo(pc2.pointcloud[j]);
            if(distance < min){
                min = distance;
            }
            // we need to calculate the exactNN for first KCenters.
//            if(distance < max){
//                break;
//            }
        }
        distances.push_back(min);
        if(min > max){
            max = min;
            if(max > currentKNN){
                max = -1; // should we change to max = -1 to denote early break? since this distance will not be used anymore
                break;
            }
        }
    }
    return maxAndDistances(max, distances);
}

double ExactHausdorff::PAMI2015_Log(PointCloud &pc1, PointCloud &pc2, bool pruning, double kthValue){
    double max = 0;
    double min = std::numeric_limits<double>::infinity(); // infinity
    double distance = 0;
    
    vector<Point> *pointcloud1 = &pc1.pointcloud;
    vector<Point> *pointcloud2 = &pc2.pointcloud;
    
    long size1 = pointcloud1->size();
    long size2 = pointcloud2->size();
    
//    double logp = log(size2+1); // +1 to avoid zero
    double logp = ceil(log10(size2+1));
    double HDLog = 0;
    
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
            max = min; // here using the logp item
            HDLog = max*logp;
            if(pruning && HDLog >= kthValue){
                max = -1; // usig -1 to denote early break
                return max;
            }
        }
        
    }
    return HDLog; // the Hausdorff distance
}

double ExactHausdorff::LowerboundFromKCenterToBound(PointCloud& pc, pair<Point, Point> &bound, double kthValue){
    
    double max = 0;
    double distance = 0;
    int kcenterCount = pc.getKCenterNum();
    for(int i = 0; i < kcenterCount; i++){
        distance = distanceFromPointToMBR(pc.pointcloud[i], bound);
        if(distance > max){
            max = distance;
            if(max >= kthValue){
                return max;
            }
        }
    }
    return max;
}

#endif /* ExactHausdorff_hpp */
