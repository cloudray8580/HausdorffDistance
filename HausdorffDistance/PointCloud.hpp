//
//  PointCloud.hpp
//  HausdorffDistance
//
//  Created by 李喆 on 2018/1/3.
//  Copyright © 2018年 李喆. All rights reserved.
//

#ifndef PointCloud_hpp
#define PointCloud_hpp

#include <stdio.h>
#include <fstream>
#include <vector>
#include "Point.hpp"
#include <limits>

#include <boost/algorithm/string.hpp>
#include "BoostRTreeSetting.hpp"
using namespace std;

class PointCloud{
public:
    PointCloud(){pointcloud.clear(); this->dimension = 3;};
    PointCloud(const vector<Point> &pointcloud);
    PointCloud(const vector<vector<double>> &pointcloud);
    PointCloud(string filename, int dimension=3); // should support .pts and .off
    PointCloud(int size, double minx, double maxx, double miny, double maxy); // random generate points
    PointCloud(PointCloud &pc, int size);
    
    vector<Point> getPoints_1();
    vector<vector<double>> getPoints_2();
    
    vector<Point> pointcloud;
    vector<Point> z_pointcloud;
    pair<Point, Point> bound;
    vector<pair<Point,Point>> FirstLevelMBRs; // actually more than first, if none, there will be bound in it
    
    bool isAvailable = true;
    int dimension;
    
    void randomize();
    
    // negative -> positive and double -> integer
    // magnitude: *10^magnitude for each coordinate value
    void prezorder(int magnitude=3); // operate on z_pointcloud 2^10 ~= 10^3, 10 bits
    // about z-order : http://www.forceflow.be/2013/10/07/morton-encodingdecoding-through-bit-interleaving-implementations/
    void zorder(); // after prezorder, each coordinate value should less than 2^21, as each coordinate only consider the first 21 bits
    
    void printAll();
    void storeToFile(string filename);
    
    void generateBoundAndMBRs(int expectedNumber);
    void sortByKcenter();
    void sortByKcenter2();
};

PointCloud::PointCloud(const vector<Point> &pointcloud){
    this->pointcloud = pointcloud;
    this->dimension = 3;
}

PointCloud::PointCloud(const vector<vector<double>> &pointcloud){
    this->pointcloud.clear();
    for (int i = 0; i < pointcloud.size(); i++){
        this->pointcloud.push_back(Point(pointcloud[i][0],pointcloud[i][1],pointcloud[i][2],i));
    }
    this->dimension = 3;
}

PointCloud::PointCloud(string filename, int dimension){
    
    vector<Point> pointcloud;
    
    ifstream infile;
    infile.open(filename);
    if(!infile)
    {
        cout << "fail to open file " << filename << endl;
    }
    
    string str;
    std::string::size_type pos1, pos2;
    const string space = " ";
    
    int count = 0;
    this->pointcloud.clear();
    
    vector<string> fields;
    split(fields, filename, boost::is_any_of("."));
    if(fields[fields.size()-1] == "pts"){
        while(getline(infile,str))   //按行读取,遇到换行符结束
        {
            //cout<<str<<endl;
            
            vector<double> temp;
            
            pos2 = str.find(space);
            pos1 = 0;
            while(std::string::npos != pos2)
            {
                temp.push_back(atof((str.substr(pos1, pos2-pos1)).c_str()));
                
                pos1 = pos2 + space.size();
                pos2 = str.find(space, pos1);
            }
            if(pos1 != str.length())
                temp.push_back(atof((str.substr(pos1)).c_str()));
            
            if(dimension == 3){
                Point point(temp[0],temp[1],temp[2], count);
                point.dimension = 3;
                pointcloud.push_back(point);
            } else if(dimension == 2){
                Point point(temp[0],temp[1]);
                point.dimension = 2;
                pointcloud.push_back(point);
            }
            count++;
        }
    } else if(fields[fields.size()-1] == "off"){
        getline(infile,str);
        getline(infile,str);
        int numberOfPoints;
        vector<string> info;
        split(info, str, boost::is_any_of(" "));
        numberOfPoints = std::stoi(info[0]);
        for(int i = 0; i < numberOfPoints; i++){
            getline(infile, str);
            vector<double> temp;
            
            pos2 = str.find(space);
            pos1 = 0;
            while(std::string::npos != pos2)
            {
                temp.push_back(atof((str.substr(pos1, pos2-pos1)).c_str()));
                
                pos1 = pos2 + space.size();
                pos2 = str.find(space, pos1);
            }
            if(pos1 != str.length())
                temp.push_back(atof((str.substr(pos1)).c_str()));
            
            Point point(temp[0],temp[1],temp[2], count);
            pointcloud.push_back(point);
            count++;
        }
        
    } else {
        cout << "unsupport file format " << fields[fields.size()-1] << endl;
    }
    
    infile.close();
    this->pointcloud = pointcloud;
    if(dimension == 3){
        this->dimension = 3;
    } else if(dimension == 2){
        this->dimension = 2;
    }
}

PointCloud::PointCloud(int size, double minx, double maxx, double miny, double maxy){
    
    double xrange = maxx-minx;
    double yrange = maxy-miny;
    
    srand((unsigned)time(NULL));
    double _x, _y;
    for(int i = 0; i < size; i++){
        _x = ((double) rand() / (RAND_MAX)) * xrange + minx;
        _y = ((double) rand() / (RAND_MAX)) * yrange + miny;
        Point p(_x, _y);
        pointcloud.push_back(p);
    }
}

PointCloud::PointCloud(PointCloud &pc, int size){
    if(size > pc.pointcloud.size()){
        size = pc.pointcloud.size();
    }
    for(int i = 0; i < size; i++){
        pointcloud.push_back(pc.pointcloud[i]);
    }
}

void PointCloud::randomize(){
    
    long size = this->pointcloud.size();
    int randomIndex = 0;
    srand((unsigned)time(NULL));
    Point temp;
    for (int i = 0; i < size; i++){
        // create random variable in [0, size)
        randomIndex = rand()%size;
        // random exchange
        temp = pointcloud[i];
        pointcloud[i] = pointcloud[randomIndex];
        pointcloud[randomIndex] = temp;
    }
}

void PointCloud::prezorder(int magnitude){
    
    z_pointcloud = pointcloud;
    
    // change the following to z_pointcloud
    
    // find the shift value
    long size = this->z_pointcloud.size();
    double minx = std::numeric_limits<double>::infinity();
    double miny = std::numeric_limits<double>::infinity();
    double minz;
    if(dimension == 3){
        minz = std::numeric_limits<double>::infinity();
    }
    for(int i = 0; i < size; i++){
        if(z_pointcloud[i].x < minx){
            minx = z_pointcloud[i].x;
        }
        if(z_pointcloud[i].y < miny){
            miny = z_pointcloud[i].y;
        }
        if(dimension == 3 & z_pointcloud[i].z < minz){
            minz = z_pointcloud[i].z;
        }
    }
    
    // turn to positive
    minx = fabs(minx);
    miny = fabs(miny);
    if(dimension == 3)
        minz = fabs(minz);
    
    // add shift value to each point
    for(int i = 0; i < size; i++){
        z_pointcloud[i].x += minx;
        z_pointcloud[i].y += miny;
        if(dimension == 3)
            z_pointcloud[i].z += minz;
        
        // multiply by 10^magnitude and only keep the integer part
        z_pointcloud[i].x *= pow(10, magnitude);
        z_pointcloud[i].y *= pow(10, magnitude);
        if(dimension == 3)
            z_pointcloud[i].z *= pow(10, magnitude);
        
        z_pointcloud[i].x = floor(z_pointcloud[i].x);
        z_pointcloud[i].y = floor(z_pointcloud[i].y);
        if(dimension == 3)
            z_pointcloud[i].z = floor(z_pointcloud[i].z);
    }
}

// ascending order
bool cmp_zorder(Point &p1, Point &p2){
    return p1.zorder < p2.zorder;
}

// according the zordered z_pointcloud, adjust the pointcloud order
// after prezorder, each coordinate should be less than 2^21, about 2*10^6
void PointCloud::zorder(){
    
    // need to do prezorder() for dataset containing negative or float value
    
    // compute zorder
    if(dimension == 3){
        long size = this->z_pointcloud.size();
        unsigned int _x;
        unsigned int _y;
        unsigned int _z;
        uint64_t zorderValue = 0;
        for(int i = 0; i < size; i++){
            zorderValue = 0;
            for(uint64_t j = 0; j < (sizeof(uint64_t)*CHAR_BIT)/3; j++){
                _x = z_pointcloud[i].x;
                _y = z_pointcloud[i].y;
                _z = z_pointcloud[i].z;
                // about priority :  */+-  higher than  << >>  higher than | &
                zorderValue |= ((_x & ((uint64_t)1 << j)) << 2*j) | ((_y & ((uint64_t)1 << j)) << (2*j+1)) | ((_z & ((uint64_t)1 << j)) << (2*j+2));
            }
            pointcloud[i].zorder = zorderValue;
        }
    } else if(dimension == 2){
        long size = this->z_pointcloud.size();
        unsigned int _x;
        unsigned int _y;
        uint64_t zorderValue = 0;
        for(int i = 0; i < size; i++){
            zorderValue = 0;
            for(uint64_t j = 0; j < (sizeof(uint64_t)*CHAR_BIT)/2; j++){
                _x = z_pointcloud[i].x;
                _y = z_pointcloud[i].y;
                // about priority :  */+-  higher than  << >>  higher than | &
                zorderValue |= ((_x & ((uint64_t)1 << j)) << 2*j) | ((_y & ((uint64_t)1 << j)) << (2*j+1));
            }
            pointcloud[i].zorder = zorderValue;
        }
    }
    // sort the pointcloud according to zorder
    sort(this->pointcloud.begin(), this->pointcloud.end(), cmp_zorder);
}

void PointCloud::printAll(){
    for(int i = 0; i < this->pointcloud.size(); i++){
        cout << i << " : " << pointcloud[i].x << " " << pointcloud[i].y << " " << pointcloud[i].z << endl;
    }
}

vector<Point> PointCloud::getPoints_1(){
    return this->pointcloud;
}

vector<vector<double>> PointCloud::getPoints_2(){
    vector<vector<double>> _pointcloud;
    
    for(int i = 0; i < this->pointcloud.size(); i++){
        vector<double> temp;
        temp.push_back(this->pointcloud[i].x);
        temp.push_back(this->pointcloud[i].y);
        temp.push_back(this->pointcloud[i].z);
        _pointcloud.push_back(temp);
    }
    return _pointcloud;
}

void PointCloud::storeToFile(string filename){
    ofstream outfile;
    outfile.open(filename);
    if(dimension == 2){
        for(int i = 0; i < pointcloud.size(); i++){
            outfile << pointcloud[i].x << " " << pointcloud[i].y << endl;
        }
    } else if(dimension == 3){
        for(int i = 0; i < pointcloud.size(); i++){
            outfile << pointcloud[i].x << " " << pointcloud[i].y << " " << pointcloud[i].z << endl;
        }
    }
}

void PointCloud::generateBoundAndMBRs(int expectedNumber){
    
    rtree refRTree(pointcloud.begin(), pointcloud.end());
    
    // calculate bound
    RTV::box_type refbound = refRTree.bounds();
    Point p1(refbound.m_min_corner.m_values[0], refbound.m_min_corner.m_values[1]);
    p1.dimension = 2;
    Point p2(refbound.m_max_corner.m_values[0], refbound.m_max_corner.m_values[1]);
    p2.dimension = 2;
    this->bound = pair<Point, Point>(p1, p2);
    
    // calculate MBRs
    vector<RTV::box_type> refVec = getMBRs2(refRTree, expectedNumber); // extract the first level
    vector<pair<Point, Point>> refMBRs;
    for(int i = 0; i < refVec.size(); i++){
        Point p1(refVec[i].m_min_corner.m_values[0], refVec[i].m_min_corner.m_values[1]);
        p1.dimension = 2;
        Point p2(refVec[i].m_max_corner.m_values[0], refVec[i].m_max_corner.m_values[1]);
        p2.dimension = 2;
        refMBRs.push_back(pair<Point, Point>(p1, p2));
    }
    // if there are no first level MBRs
    if(refMBRs.size() == 0){
        refMBRs.push_back(pair<Point, Point>(this->bound));
    }
    this->FirstLevelMBRs = refMBRs;
}

void PointCloud::sortByKcenter(){
    
    if(pointcloud.size() <= 3)
        return;
    
    long num = this->pointcloud.size();
    num *= 0.05;
    if(num > 100){
        num = 100;
    }
    if(num < 10){
        num = 10;
    }
    if(num > pointcloud.size()){
        num = pointcloud.size();
    }
    
    vector<Point> kcenters;
    
    double **distanceMatrix = new double *[pointcloud.size()];
//    vector<vector<double>> distanceMatrix;
//    vector<double> subDistanceMatrix;
    int diameterIndex1 = 0, diameterIndex2 = 1;
    double distance;
    double diameter = 0;
    
    // first, calculate the vector to store the distance of each pair of points
    for (int i1 = 0; i1 < pointcloud.size(); i1++){
        pointcloud[i1].index = i1;
//        subDistanceMatrix.clear();
        distanceMatrix[i1] = new double[pointcloud.size()];
        for (int i2 = 0; i2 < pointcloud.size(); i2++){
            if(i1 == i2){
                distance = 0;
            } else {
                distance = pointcloud[i1].distanceTo(pointcloud[i2]);
            }
//            subDistanceMatrix.push_back(distance);
            distanceMatrix[i1][i2] = distance; // this will induce SIG KILL, memory 57 GB
            if (distance > diameter){
                diameter = distance;
                diameterIndex1 = i1;
                diameterIndex2 = i2;
            }
        }
//        distanceMatrix.push_back(subDistanceMatrix);
    }
    
    // add the diameter point as the initial 2 centers.
    // this code will have a bug if the one of the diameter is at the end but you do not delete it first.
    if(diameterIndex1 == pointcloud.size()-1){
        pointcloud[diameterIndex1].isCenter = true;
        kcenters.push_back(pointcloud[diameterIndex1]);
        pointcloud[diameterIndex1] = pointcloud.back();
        pointcloud.pop_back();
        
        pointcloud[diameterIndex2].isCenter = true;
        kcenters.push_back(pointcloud[diameterIndex2]);
        pointcloud[diameterIndex2] = pointcloud.back();
        pointcloud.pop_back();
    } else {
        pointcloud[diameterIndex2].isCenter = true;
        kcenters.push_back(pointcloud[diameterIndex2]);
        pointcloud[diameterIndex2] = pointcloud.back();
        pointcloud.pop_back();
        
        pointcloud[diameterIndex1].isCenter = true;
        kcenters.push_back(pointcloud[diameterIndex1]);
        pointcloud[diameterIndex1] = pointcloud.back();
        pointcloud.pop_back();
    }
    
//    num = pointcloud.size();
    num -= 2;
    double maxMinDistance = 0;
    double mindistance = 0;
    double targetIndex = 0;
    // for each remaining point, find the max(min distance to current kcenters)
    while(num){
        maxMinDistance = 0;
        targetIndex = 0;
        for(int i = 0; i < pointcloud.size(); i++){
            mindistance = distanceMatrix[pointcloud[i].index][kcenters[0].index];
            for(int j = 0; j < kcenters.size(); j++){
                distance = distanceMatrix[pointcloud[i].index][kcenters[j].index];
                if(distance < mindistance){
                    mindistance = distance;
                }
            }
            if(mindistance > maxMinDistance){
                maxMinDistance = mindistance;
                targetIndex = i;
            }
        }
        kcenters.push_back(pointcloud[targetIndex]);
        pointcloud[targetIndex] = pointcloud.back();
        pointcloud.pop_back();
        num--;
        if(num % 10 == 0)
            cout << "num: " << num << endl;
    }
   
    std::copy(pointcloud.begin(), pointcloud.end(), std::back_inserter(kcenters));
    this->pointcloud = kcenters;
    
    // deallocate memory
    for(int i = 0; i < pointcloud.size(); i++){
        delete [] distanceMatrix[i];
    }
    delete [] distanceMatrix;
//    cout << "lala";
}

// do not use distanceMatrix as it will be too large!
void PointCloud::sortByKcenter2(){
    if(pointcloud.size() <= 3)
        return;
    
    long num = this->pointcloud.size();
    num *= 0.05;
    if(num > 100){
        num = 100;
    }
    if(num < 10){
        num = 10;
    }
    if(num > pointcloud.size()){
        num = pointcloud.size();
    }
    
    vector<Point> kcenters;
    
    int diameterIndex1 = 0, diameterIndex2 = 1;
    double distance;
    double diameter = 0;
    
    // if the pointcloud is too large, calculating its diameter will be too costly
    // we random choose a point as randomIndex1
    if (pointcloud.size() > 10000){
        srand((unsigned)time(NULL));
        diameterIndex1 = rand()%pointcloud.size();
        for(int i = 0; i < pointcloud.size(); i++){
            distance = pointcloud[diameterIndex1].distanceTo(pointcloud[i]);
            if (distance > diameter){
                diameter = distance;
                diameterIndex2 = i;
            }
        }
    } else {
        // calculate its diameter points
        for (int i1 = 0; i1 < pointcloud.size(); i1++){
            for (int i2 = i1; i2 < pointcloud.size(); i2++){
                if(i1 == i2){
                    distance = 0;
                } else {
                    distance = pointcloud[i1].distanceTo(pointcloud[i2]);
                }
                if (distance > diameter){
                    diameter = distance;
                    diameterIndex1 = i1;
                    diameterIndex2 = i2;
                }
            }
        }
    }
    
    
    // add the diameter point as the initial 2 centers.
    // this code will have a bug if the one of the diameter is at the end but you do not delete it first.
    if(diameterIndex1 == pointcloud.size()-1){
        pointcloud[diameterIndex1].isCenter = true;
        kcenters.push_back(pointcloud[diameterIndex1]);
        pointcloud[diameterIndex1] = pointcloud.back();
        pointcloud.pop_back();
        
        pointcloud[diameterIndex2].isCenter = true;
        kcenters.push_back(pointcloud[diameterIndex2]);
        pointcloud[diameterIndex2] = pointcloud.back();
        pointcloud.pop_back();
    } else {
        pointcloud[diameterIndex2].isCenter = true;
        kcenters.push_back(pointcloud[diameterIndex2]);
        pointcloud[diameterIndex2] = pointcloud.back();
        pointcloud.pop_back();
        
        pointcloud[diameterIndex1].isCenter = true;
        kcenters.push_back(pointcloud[diameterIndex1]);
        pointcloud[diameterIndex1] = pointcloud.back();
        pointcloud.pop_back();
    }
    
    num -= 2;
    double maxMinDistance = 0;
    double mindistance = 0;
    double targetIndex = 0;
    // for each remaining point, find the max(min distance to current kcenters)
    while(num){
        maxMinDistance = 0;
        targetIndex = 0;
        for(int i = 0; i < pointcloud.size(); i++){
            mindistance = pointcloud[i].distanceTo(kcenters[0]);
            for(int j = 0; j < kcenters.size(); j++){
                distance = pointcloud[i].distanceTo(kcenters[j]);
                if(distance < mindistance){
                    mindistance = distance;
                }
            }
            if(mindistance > maxMinDistance){
                maxMinDistance = mindistance;
                targetIndex = i;
            }
        }
        kcenters.push_back(pointcloud[targetIndex]);
        pointcloud[targetIndex] = pointcloud.back();
        pointcloud.pop_back();
        num--;
        if(num % 10 == 0)
            cout << "num: " << num << endl;
    }
    
    std::copy(pointcloud.begin(), pointcloud.end(), std::back_inserter(kcenters));
    this->pointcloud = kcenters;
    
    cout << "lala";
}

#endif /* PointCloud_hpp */
