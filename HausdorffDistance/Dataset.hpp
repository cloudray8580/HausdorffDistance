//
//  Dataset.hpp
//  HausdorffDistance
//
//  Created by 李喆 on 2018/1/12.
//  Copyright © 2018年 李喆. All rights reserved.
//

#ifndef Dataset_hpp
#define Dataset_hpp
#include "PointCloud.hpp"
#include <vector>
#include <stdio.h>

using namespace std;

class Dataset{
public:
    static vector<PointCloud> GeneratePointCloudFromNationalFile(string filepath);
    static vector<PointCloud> GeneratePointCloudFromPOIFile(string filepath);
    static vector<PointCloud> GeneratePointCloudFromTwitterFile(string filepath);
    static bool StorePointCloudIntoFile(string filepath, vector<PointCloud> &dataset);
    static vector<PointCloud> RestorePointCloudFromFile(string filepath);
};

// need to handle 2-dimension point
vector<PointCloud> Dataset::GeneratePointCloudFromNationalFile(string filepath){
    vector<PointCloud> dataset;
    
    ifstream infile;
    infile.open(filepath);
    string str;
    // read the file header info
    getline(infile,str);
    cout << str << endl;
    vector<string> fields;
    
    string featureClass;
    string latitude_str;
    string longitude_str;
    double latitude;
    double longitude;
    
    map<string, PointCloud> dataset_feature;
    
    int lines = 0;
    while(getline(infile,str)){
        lines++;
        
        fields.clear();
        split(fields, str, boost::is_any_of("|"));
        featureClass = fields[1];
        latitude_str = fields[9];
        longitude_str = fields[10];
        
        try{
            latitude = stod(latitude_str);
            longitude = stod(longitude_str);
        } catch (std::invalid_argument e)
        {
            cout << "' --> <invalid_argument>" << endl;
            continue;
        }
        catch (std::out_of_range e)
        {
            cout << "' --> <out_of_range>" << endl;
            continue;
        }

        
        Point point = Point(latitude, longitude);
        point.dimension = 2;
        if(dataset_feature.find(featureClass) == dataset_feature.end()){
            PointCloud pc = PointCloud();
            pc.dimension = 2;
            pc.pointcloud.push_back(point);
            dataset_feature.insert(pair<string, PointCloud>(featureClass, pc));
        } else {
            dataset_feature[featureClass].pointcloud.push_back(point);
        }
        
        if(lines % 10000 == 0){
            cout << lines << endl;
        }
        if (lines == 1800001){
            break;
        }
    }
    
    map<int, int> countsize;
    long size;
    int count = 1;
    for(auto it = dataset_feature.begin(); it != dataset_feature.end();it++){
        //cout << count << " " << it->second.pointcloud.size() << endl;
        size = it->second.pointcloud.size();
        if(size > 100)
            countsize[100]++;
        else
            countsize[size/10]++;
        count++;
        dataset.push_back(it->second);
    }
    
    for (auto it = countsize.begin(); it != countsize.end(); it++){
        cout << it->first << " " << it->second << endl;
    }
    return dataset;
}

vector<PointCloud> Dataset::GeneratePointCloudFromPOIFile(string filepath){
    vector<PointCloud> dataset;
    
    ifstream infile;
    infile.open(filepath);
    vector<string> fields;
    string str;
    
    string coordinate;
    string feature;
    vector<string> features;
    string latitude_str;
    string longitude_str;
    double latitude;
    double longitude;
    map<string, PointCloud> dataset_feature;
    int lines = 0;
    while(getline(infile,str)){
        lines++;
        fields.clear();
        split(fields, str, boost::is_any_of("\t"));
        coordinate = fields[1];
        feature = fields.back(); // get the last element
        
        split(fields, coordinate, boost::is_any_of(","));
        latitude_str = fields[0];
        longitude_str = fields[1];
        latitude = stod(latitude_str);
        longitude = stod(longitude_str);
        
//        split(fields, feature, boost::is_any_of(" "));
//        feature = fields.back(); // get the last of the last
        
//        split(fields, str, boost::is_any_of("\t"));
//        cout << latitude << " " << longitude << " ";
//        for(int i = 2; i < fields.size(); i++){
//            cout << fields[i] << " ";
//        }
//        cout << endl;
//
//        if(lines == 10)
//            break;
        
        Point point = Point(latitude, longitude);
        point.dimension = 2;
        if(dataset_feature.find(feature) == dataset_feature.end()){
            PointCloud pc = PointCloud();
            pc.dimension = 2;
            pc.pointcloud.push_back(point);
            dataset_feature.insert(pair<string, PointCloud>(feature, pc));
        } else {
            dataset_feature[feature].pointcloud.push_back(point);
        }

//        cout << latitude << " " << longitude << " " << feature << endl;

        if(lines%10000 == 0){
            cout << lines << endl;
        }
    }
    long min = 100000000;
    long max = 0;
    long totalpoints = 0;
    long totalpointclouds = 0;
    for(auto it = dataset_feature.begin(); it != dataset_feature.end(); it++){
        cout << it->first << " " << it->second.pointcloud.size() << endl;
        dataset.push_back(it->second);
        if(it->second.pointcloud.size() < min){
            min = it->second.pointcloud.size();
        }
        if(it->second.pointcloud.size() > max){
            max = it->second.pointcloud.size();
        }
        totalpoints += it->second.pointcloud.size();
        totalpointclouds += 1;
    }
    cout << ">>>>>>>>> size : " << dataset.size() << " max: "<< max << "  min: " << min << "  Average points: " << totalpoints/totalpointclouds << " totalpoints:" << totalpoints << "  totalpointclouds: " << totalpointclouds << endl;
    return dataset;
}

vector<PointCloud> Dataset::GeneratePointCloudFromTwitterFile(string filepath){
    vector<PointCloud> dataset;
    
    ifstream infile;
    infile.open(filepath);
    vector<string> fields;
    string str;
    
    string coordinate;
    string feature;
    vector<string> features;
    string latitude_str;
    string longitude_str;
    double latitude;
    double longitude;
    map<string, PointCloud> dataset_feature;
    int lines = 0;
    while(getline(infile,str)){
        lines++;
        fields.clear();
        split(fields, str, boost::is_any_of("\t"));
        feature = fields[1];
        latitude_str = fields[2];
        longitude_str = fields[3];
        latitude = stod(latitude_str);
        longitude = stod(longitude_str);
        split(features, feature, boost::is_any_of(","));
        
        Point point = Point(latitude, longitude);
        point.dimension = 2;
        for (int i = 0; i < features.size(); i++){
            feature = features[i];
            if(dataset_feature.find(feature) == dataset_feature.end()){
                PointCloud pc = PointCloud();
                pc.dimension = 2;
                pc.pointcloud.push_back(point);
                dataset_feature.insert(pair<string, PointCloud>(feature, pc));
            } else {
                dataset_feature[feature].pointcloud.push_back(point);
            }
        }
        
        if(lines%10000 == 0){
            cout << lines << endl;
        }
//        if (lines == 1000000){
//            break;
//        }
    }
    
    long min = 100000000;
    long max = 0;
    long totalpoints = 0;
    long totalpointclouds = 0;
    for(auto it = dataset_feature.begin(); it != dataset_feature.end(); it++){
        cout << it->first << " " << it->second.pointcloud.size() << endl;
        dataset.push_back(it->second);
        if(it->second.pointcloud.size() < min){
            min = it->second.pointcloud.size();
        }
        if(it->second.pointcloud.size() > max){
            max = it->second.pointcloud.size();
        }
        totalpoints += it->second.pointcloud.size();
        totalpointclouds += 1;
    }
    cout << ">>>>>>>>> size : " << dataset.size() << " max: "<< max << "  min: " << min << "  Average points: " << totalpoints/totalpointclouds << " totalpoints:" << totalpoints << "  totalpointclouds: " << totalpointclouds << endl;
    
    return dataset;
}

bool Dataset::StorePointCloudIntoFile(string filepath, vector<PointCloud> &dataset){
    bool isSuccess = false;
    
    ofstream outfile;
    outfile.open(filepath);
    
    for(int i = 0; i < dataset.size(); i++){
        for(int j = 0; j < dataset[i].pointcloud.size(); j++){
            outfile << dataset[i].pointcloud[j].x << " " << dataset[i].pointcloud[j].y << endl;
        }
        outfile << "====" << endl;
    }
    isSuccess = true;
    return isSuccess;
}

vector<PointCloud> Dataset::RestorePointCloudFromFile(string filepath){
    vector<PointCloud> dataset;
    
    ifstream infile;
    infile.open(filepath);
    string str;
    vector<string> coordinates;
    double xcor, ycor;
    PointCloud pc = PointCloud();
    int count = 0;
    while(getline(infile, str)){
        count++;
        if(count %10000 == 0){
            cout << count <<endl;
        }
        if(str == "===="){
            dataset.push_back(pc);
            pc.pointcloud.clear();
        } else {
            split(coordinates, str, boost::is_any_of(" "));
            xcor = stod(coordinates[0]);
            ycor = stod(coordinates[1]);
            Point point = Point(xcor, ycor);
            pc.pointcloud.push_back(point);
        }
    }
    return dataset;
}

#endif /* Dataset_hpp */
