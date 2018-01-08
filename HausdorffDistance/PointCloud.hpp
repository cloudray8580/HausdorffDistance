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
#endif /* PointCloud_hpp */
using namespace std;

class PointCloud{
public:
    PointCloud(vector<Point> pointcloud);
    PointCloud(vector<vector<double>> pointcloud);
    PointCloud(string filename);
    
    vector<Point> pointcloud;
    vector<Point> z_pointcloud;
    
    int dimension;
    
    void randomize();
    // negative -> positive and double -> integer
    // magnitude: *10^magnitude for each coordinate value
    void prezorder(int magnitude=3); // operate on z_pointcloud 2^10 ~= 10^3, 10 bits
    // about z-order : http://www.forceflow.be/2013/10/07/morton-encodingdecoding-through-bit-interleaving-implementations/
    void zorder(); // after prezorder, each coordinate value should less than 2^21, as each coordinate only consider the first 21 bits
    
    void printAll();
    
    vector<Point> getPoints_1();
    vector<vector<double>> getPoints_2();
};

PointCloud::PointCloud(vector<Point> pointcloud){
    this->pointcloud = pointcloud;
}

PointCloud::PointCloud(vector<vector<double>> pointcloud){
    this->pointcloud.clear();
    for (int i = 0; i < pointcloud.size(); i++){
        this->pointcloud.push_back(Point(pointcloud[i][0],pointcloud[i][1],pointcloud[i][2],i));
    }
}

PointCloud::PointCloud(string filename){
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
        
        Point point(temp[0],temp[1],temp[2], count);
        pointcloud.push_back(point);
        count++;
    }
    infile.close();
    this->pointcloud = pointcloud;
}

void PointCloud::randomize(){
    
    long size = this->pointcloud.size();
    int randomIndex = 0;
    srand((unsigned)time(NULL));
    Point temp;
    for (int i = 0; i < size; i++){
        // create random variable in [0, size)
        randomIndex = rand()%size;
        // tandom exchange
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
    double minz = std::numeric_limits<double>::infinity();
    for(int i = 0; i < size; i++){
        if(z_pointcloud[i].x < minx){
            minx = z_pointcloud[i].x;
        }
        if(z_pointcloud[i].y < miny){
            miny = z_pointcloud[i].y;
        }
        if(z_pointcloud[i].z < minz){
            minz = z_pointcloud[i].z;
        }
    }
    
    // turn to positive
    minx = fabs(minx);
    miny = fabs(miny);
    minz = fabs(minz);
    
    // add shift value to each point
    for(int i = 0; i < size; i++){
        z_pointcloud[i].x += minx;
        z_pointcloud[i].y += miny;
        z_pointcloud[i].z += minz;
        
        // multiply by 10^magnitude and only keep the integer part
        z_pointcloud[i].x *= pow(10, magnitude);
        z_pointcloud[i].y *= pow(10, magnitude);
        z_pointcloud[i].z *= pow(10, magnitude);
        
        z_pointcloud[i].x = floor(z_pointcloud[i].x);
        z_pointcloud[i].y = floor(z_pointcloud[i].y);
        z_pointcloud[i].z = floor(z_pointcloud[i].z);
    }
}

// ascending order
bool cmp_zorder(Point p1, Point p2){
    return p1.zorder < p2.zorder;
}

// according the zordered z_pointcloud, adjust the pointcloud order
// after prezorder, each coordinate should be less than 2^21, about 2*10^6
void PointCloud::zorder(){
    
    // need to do prezorder() for dataset containing negative or float value
    
    // compute zorder
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
