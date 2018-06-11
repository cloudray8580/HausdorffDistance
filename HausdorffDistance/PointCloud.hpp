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

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/spatial_sort.h>
#include <CGAL/hilbert_sort.h>

#include "HilbertCurve.hpp"

using namespace std;

struct MyLessX{
    bool operator()(const Point& p, const Point& q) const{
        return p.x < q.x;
    }
};

struct MyLessY{
    bool operator()(const Point& p, const Point& q) const{
        return p.y < q.y;
    }
};

struct MySpatialSortingTraits {
    typedef Point Point_2;
    typedef MyLessX Less_x_2;
    typedef MyLessY Less_y_2;
    
    Less_x_2 less_x_2_object() const
    {
        return Less_x_2();
    }
    Less_y_2 less_y_2_object() const
    {
        return Less_y_2();
    }
};

class PointCloud{
public:
    PointCloud(){pointcloud.clear(); this->dimension = 2;};
    PointCloud(const vector<Point> &pointcloud);
    PointCloud(const vector<vector<double>> &pointcloud);
    PointCloud(string filename, int dimension=2); // should support .pts and .off
    PointCloud(int size, double minx, double maxx, double miny, double maxy); // random generate points
    PointCloud(PointCloud &pc, int size);
    
    vector<Point> getPoints_1();
    vector<vector<double>> getPoints_2();
    
    vector<Point> pointcloud;
    vector<Point> z_pointcloud;
    pair<Point, Point> bound;
    vector<pair<Point,Point>> FirstLevelMBRs; // actually more than first, if none, there will be bound in it
    
    bool isAvailable = true;
    int dimension = 2;
    string keyword;
    int keywordId;
    Point center;
    double distance = 0;
    int size = 0;
    void calculateCenterPoint();
    
    void randomize();
    
    // negative -> positive and double -> integer
    // magnitude: *10^magnitude for each coordinate value
    void prezorder(int magnitude=3); // operate on z_pointcloud 2^10 ~= 10^3, 10 bits
    // about z-order : http://www.forceflow.be/2013/10/07/morton-encodingdecoding-through-bit-interleaving-implementations/
    void zorder(); // after prezorder, each coordinate value should less than 2^21, as each coordinate only consider the first 21 bits
    
    void calculateHilbertValue();
    void myHilbertOrder();
    
    void hilbertOrder(); // using CGAL library, support double (including negative double value)
    
    void printAll();
    void storeToFile(string filename);
    
    void generateBoundAndMBRs(int expectedNumber);
    
    // without distanceMatrix, and use nearest_center_dist[i] for all point, O(nk)
    void sortByKcenter(double percent=0.05, int lowerthreshold=1, int upperthreshold=10000);
    void sortByKcenter2(); // O(nk^2)
    
    // return the distance of points to its nearest kcenter, and its kcenter index!
    vector<pair<double,int>> sortByKcenterWithRecord(double percent=0.05, int lowerthreshold=1, int upperthreshold=10000); // return the distance of points to its nearest kcenter
    
    int getKCenterNum(double percentage = 0.05, int lowerthreshold=1, int upperthreshold=10000);
    void generateRealWorldPosition(string outputfile); // from latitude-longitude to longitude-latitude
};

PointCloud::PointCloud(const vector<Point> &pointcloud){
    this->pointcloud = pointcloud;
    this->dimension = 2;
}

PointCloud::PointCloud(const vector<vector<double>> &pointcloud){
    this->pointcloud.clear();
    for (int i = 0; i < pointcloud.size(); i++){
        this->pointcloud.push_back(Point(pointcloud[i][0],pointcloud[i][1],pointcloud[i][2],i));
    }
    this->dimension = 2;
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

void PointCloud::calculateCenterPoint(){
    double sum_x = 0, sum_y = 0;
    for(int i = 0; i < pointcloud.size(); i++){
        sum_x += pointcloud[i].x;
        sum_y += pointcloud[i].y;
    }
    center = Point(sum_x/pointcloud.size(), sum_y/pointcloud.size());
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
//
//    z_pointcloud = pointcloud;
//
//    // change the following to z_pointcloud
//
//    // find the shift value
//    long size = this->z_pointcloud.size();
//    double minx = std::numeric_limits<double>::infinity();
//    double miny = std::numeric_limits<double>::infinity();
//    double minz;
//    if(dimension == 3){
//        minz = std::numeric_limits<double>::infinity();
//    }
//    for(int i = 0; i < size; i++){
//        if(z_pointcloud[i].x < minx){
//            minx = z_pointcloud[i].x;
//        }
//        if(z_pointcloud[i].y < miny){
//            miny = z_pointcloud[i].y;
//        }
//        if(dimension == 3 & z_pointcloud[i].z < minz){
//            minz = z_pointcloud[i].z;
//        }
//    }
//
//    // turn to positive
//    minx = fabs(minx);
//    miny = fabs(miny);
//    if(dimension == 3)
//        minz = fabs(minz);
//
//    // add shift value to each point
//    for(int i = 0; i < size; i++){
//        z_pointcloud[i].x += minx;
//        z_pointcloud[i].y += miny;
//        if(dimension == 3)
//            z_pointcloud[i].z += minz;
//
//        // multiply by 10^magnitude and only keep the integer part
//        z_pointcloud[i].x *= pow(10, magnitude);
//        z_pointcloud[i].y *= pow(10, magnitude);
//        if(dimension == 3)
//            z_pointcloud[i].z *= pow(10, magnitude);
//
//        z_pointcloud[i].x = floor(z_pointcloud[i].x);
//        z_pointcloud[i].y = floor(z_pointcloud[i].y);
//        if(dimension == 3)
//            z_pointcloud[i].z = floor(z_pointcloud[i].z);
//    }
}

// ascending order
bool cmp_zorder(Point &p1, Point &p2){
    return p1.zorder < p2.zorder;
}

// according the zordered z_pointcloud, adjust the pointcloud order
// after prezorder, each coordinate should be less than 2^21, about 2*10^6
void PointCloud::zorder(){
//
//    // need to do prezorder() for dataset containing negative or float value
//
//    // compute zorder
//    if(dimension == 3){
//        long size = this->z_pointcloud.size();
//        unsigned int _x;
//        unsigned int _y;
//        unsigned int _z;
//        uint64_t zorderValue = 0;
//        for(int i = 0; i < size; i++){
//            zorderValue = 0;
//            for(uint64_t j = 0; j < (sizeof(uint64_t)*CHAR_BIT)/3; j++){
//                _x = z_pointcloud[i].x;
//                _y = z_pointcloud[i].y;
//                _z = z_pointcloud[i].z;
//                // about priority :  */+-  higher than  << >>  higher than | &
//                zorderValue |= ((_x & ((uint64_t)1 << j)) << 2*j) | ((_y & ((uint64_t)1 << j)) << (2*j+1)) | ((_z & ((uint64_t)1 << j)) << (2*j+2));
//            }
//            pointcloud[i].zorder = zorderValue;
//        }
//    } else if(dimension == 2){
//        long size = this->z_pointcloud.size();
//        unsigned int _x;
//        unsigned int _y;
//        uint64_t zorderValue = 0;
//        for(int i = 0; i < size; i++){
//            zorderValue = 0;
//            for(uint64_t j = 0; j < (sizeof(uint64_t)*CHAR_BIT)/2; j++){
//                _x = z_pointcloud[i].x;
//                _y = z_pointcloud[i].y;
//                // about priority :  */+-  higher than  << >>  higher than | &
//                zorderValue |= ((_x & ((uint64_t)1 << j)) << 2*j) | ((_y & ((uint64_t)1 << j)) << (2*j+1));
//            }
//            pointcloud[i].zorder = zorderValue;
//        }
//    }
//    // sort the pointcloud according to zorder
//    sort(this->pointcloud.begin(), this->pointcloud.end(), cmp_zorder);
}

void PointCloud::calculateHilbertValue(){
    // x stands for latitude
    // y stands for longitude
//
//    // increase x by 90, increase y by 180
//    for (int i = 0; i < pointcloud.size(); i++){
//        int map_x = (pointcloud[i].x + 90) * 10000; // ignore the part after 4 decimal point
//        int map_y = (pointcloud[i].y + 180) * 10000;
////        int map_x = (pointcloud[i].x + 180);
////        int map_y = (pointcloud[i].y + 180);
////        int map_x = (pointcloud[i].x);
////        int map_y = (pointcloud[i].y);
//        double _hilbertValue = xy2d(HILBERT_N, map_x, map_y);
//        pointcloud[i].hilbertValue = _hilbertValue;
//    }
}

bool cmp_hilbertValue(Point &p1, Point& p2){
    return p1.hilbertValue < p2.hilbertValue;
}

void PointCloud::myHilbertOrder(){
    sort(pointcloud.begin(), pointcloud.end(), cmp_hilbertValue);
}

void PointCloud::hilbertOrder(){
    MySpatialSortingTraits sst;
    CGAL::hilbert_sort(this->pointcloud.begin(), this->pointcloud.end(), sst);
//    CGAL::spatial_sort(this->pointcloud.begin(), this->pointcloud.end(), sst); // by default using zorder I think
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

void PointCloud::sortByKcenter(double percent, int lowerthreshold, int upperthreshold){
    
    if(pointcloud.size() <= 3)
        return;
    
    // every time you use this, refresh isCenter to all false!
    for(int i = 0; i < pointcloud.size(); i++){
        pointcloud[i].isCenter = false;
    }
    
    long num = this->pointcloud.size();
    num *= percent;
    if(num > upperthreshold){
        num = upperthreshold;
    }
    if(num < lowerthreshold){
        num = lowerthreshold;
    }
    if(num > pointcloud.size()){
        num = pointcloud.size();
    }
    
    vector<Point> kcenters;
    
    // compute the first center, using centroid
    double totalx = 0;
    double totaly = 0;
    double NNDistanceCentroid = std::numeric_limits<double>::infinity();
    for(int i = 0; i < pointcloud.size(); i++){
        totalx += pointcloud[i].x;
        totaly += pointcloud[i].y;
    }
    Point centroid(totalx/pointcloud.size(), totaly/pointcloud.size());
    double distance = 0;
    int centerIndex = 0;
    // finding its NN
    for(int i = 0; i < pointcloud.size(); i++){
        distance = centroid.distanceTo(pointcloud[i]);
        if(distance < NNDistanceCentroid){
            NNDistanceCentroid = distance;
            centerIndex = i;
        }
    }
    pointcloud[centerIndex].isCenter = true;
    kcenters.push_back(pointcloud[centerIndex]);
//    pointcloud[centerIndex] = pointcloud.back();
//    pointcloud.pop_back();
    
    // calculate the distance for each point to its nearest kcenter
    double nearest_center_dist[pointcloud.size()];
    for(int i = 0; i < pointcloud.size(); i++){
        distance = pointcloud[i].distanceTo(kcenters[0]);
        nearest_center_dist[i] = distance;
    }
    
    num -= 1; // besides the center
    
    double maxMinDistance = 0;
    double mindistance = 0;
    double targetIndex = 0;
    // for the remaining points, find the max(min distance to current kcenters)
    // for each remaining point, calculate its min distance to the current kcenters

    while(num){
        maxMinDistance = 0;
        targetIndex = 0;
        
        // select the max NN distance
        for(int i = 0; i < pointcloud.size(); i++){
            if(pointcloud[i].isCenter)
                continue;
            if(nearest_center_dist[i] > maxMinDistance){
                maxMinDistance = nearest_center_dist[i];
                targetIndex = i;
            }
        }
        
        // add as new center
        pointcloud[targetIndex].isCenter = true;
        
        kcenters.push_back(pointcloud[targetIndex]);
        
        // update NN distance for all
        for(int i = 0; i < pointcloud.size(); i++){
            distance = pointcloud[i].distanceTo(kcenters.back());
            if(distance < nearest_center_dist[i])
                nearest_center_dist[i] = distance;
        }
        num--;
    }
    
    // delete the center points
    // the reason why use this is avoid the back is also center
    int ksize = kcenters.size();
    int currentIndex = 0;
    
    while(ksize){
        while(pointcloud[currentIndex].isCenter && ksize){
            pointcloud[currentIndex] = pointcloud.back();
            pointcloud.pop_back();
            ksize--;
        }
        currentIndex++;
    }
   
    std::copy(pointcloud.begin(), pointcloud.end(), std::back_inserter(kcenters));
    this->pointcloud = kcenters;
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
//        if(num % 10 == 0)
//            cout << "num: " << num << endl;
    }
    
    std::copy(pointcloud.begin(), pointcloud.end(), std::back_inserter(kcenters));
    this->pointcloud = kcenters;
    
    cout << "lala";
}

vector<pair<double,int>> PointCloud::sortByKcenterWithRecord(double percent, int lowerthreshold, int upperthreshold){
    if(pointcloud.size() <= 3){
        vector<pair<double,int>> result1;
        result1.push_back(pair<double,int>(0,0));
        result1.push_back(pair<double,int>(0,1));
        result1.push_back(pair<double,int>(0,2));
        return result1;
    }
    
    // every time you use this, refresh isCenter to all false!
    for(int i = 0; i < pointcloud.size(); i++){
        pointcloud[i].isCenter = false;
    }
    
    long num = this->pointcloud.size();
    num *= percent;
    if(num > upperthreshold){
        num = upperthreshold;
    }
    if(num < lowerthreshold){
        num = lowerthreshold;
    }
    if(num > pointcloud.size()){
        num = pointcloud.size();
    }
    
    vector<Point> kcenters;
    
    // compute the first center, using centroid
    double totalx = 0;
    double totaly = 0;
    double NNDistanceCentroid = std::numeric_limits<double>::infinity();
    for(int i = 0; i < pointcloud.size(); i++){
        totalx += pointcloud[i].x;
        totaly += pointcloud[i].y;
    }
    Point centroid(totalx/pointcloud.size(), totaly/pointcloud.size());
    double distance = 0;
    int centerIndex = 0;
    // finding its NN
    for(int i = 0; i < pointcloud.size(); i++){
        distance = centroid.distanceTo(pointcloud[i]);
        if(distance < NNDistanceCentroid){
            NNDistanceCentroid = distance;
            centerIndex = i;
        }
    }
    pointcloud[centerIndex].isCenter = true;
    kcenters.push_back(pointcloud[centerIndex]);
    //    pointcloud[centerIndex] = pointcloud.back();
    //    pointcloud.pop_back();
    
    // calculate the distance for each point to its nearest kcenter
    vector<pair<double,int>> nearest_center_dist;
    for(int i = 0; i < pointcloud.size(); i++){
        distance = pointcloud[i].distanceTo(kcenters[0]);
        nearest_center_dist.push_back(pair<double,int>(distance,0));
    }
    
    num -= 1; // besides the center
    
    double maxMinDistance = 0;
    double mindistance = 0;
    double targetIndex = 0;
    // for the remaining points, find the max(min distance to current kcenters)
    // for each remaining point, calculate its min distance to the current kcenters
    while(num){
        maxMinDistance = 0;
        targetIndex = 0;
        
        // select the max NN distance
        for(int i = 0; i < pointcloud.size(); i++){
            if(pointcloud[i].isCenter)
                continue;
            if(nearest_center_dist[i].first > maxMinDistance){
                maxMinDistance = nearest_center_dist[i].first;
                targetIndex = i;
            }
        }
        
        // add as new center
        pointcloud[targetIndex].isCenter = true;
        kcenters.push_back(pointcloud[targetIndex]);
        
        // update NN distance for all
        for(int i = 0; i < pointcloud.size(); i++){
            distance = pointcloud[i].distanceTo(kcenters.back());
            if(distance < nearest_center_dist[i].first){
                nearest_center_dist[i].first = distance;
                nearest_center_dist[i].second = kcenters.size()-1;
            }
        }
        num--;
    }
    
    // delete the center points
    // the reason why use this is avoid the back is also center
    int ksize = kcenters.size();
    int currentIndex = 0;
    while(ksize){
        while(pointcloud[currentIndex].isCenter && ksize){
            pointcloud[currentIndex] = pointcloud.back();
            pointcloud.pop_back();
            nearest_center_dist[currentIndex] = nearest_center_dist.back();
            nearest_center_dist.pop_back();
            ksize--;
        }
        currentIndex++;
    }
    
    std::copy(pointcloud.begin(), pointcloud.end(), std::back_inserter(kcenters));
    this->pointcloud = kcenters;
    
    return nearest_center_dist;
}

int PointCloud::getKCenterNum(double percentage, int lowerthreshold, int upperthreshold){
    long num = this->pointcloud.size();
    num *= percentage;
    if(num > upperthreshold){
        num = upperthreshold;
    }
    if(num < lowerthreshold){
        num = lowerthreshold;
    }
    if(num > pointcloud.size()){
        num = pointcloud.size();
    }
    return num;
}

void PointCloud::generateRealWorldPosition(string outputfile){
    for(int i = 0; i < pointcloud.size(); i++){
        double temp = pointcloud[i].x;
        pointcloud[i].x = pointcloud[i].y;
        pointcloud[i].y = temp;
    }
    this->storeToFile(outputfile);
}

#endif /* PointCloud_hpp */
