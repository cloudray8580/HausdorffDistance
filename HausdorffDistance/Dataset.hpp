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
#include <regex>
#include <boost/algorithm/string.hpp>
using namespace std;

class Dataset{
public:
    static vector<PointCloud> GeneratePointCloudFromNationalFile(string filepath);
    static vector<PointCloud> GeneratePointCloudFromPOIFile(string filepath);
    static vector<PointCloud> GeneratePointCloudFromTwitterFile(string filepath);
    static map<string, PointCloud> GeneratePointCloudFromTwitterFileWithKeyword(string filepath);
    static vector<PointCloud> GeneratePointCloudFromTwitterFileWithKeyword2(string filepath);
    static vector<Point> GetAllPoints(string filepath);
    
    static void GenerateTweetPointCloudsAndAllPoints(string filepath, vector<PointCloud> &pcs, vector<Point> &ps);
    
    static map<string, PointCloud> GeneratePointCloudFromTwitterFileWithKeywordUSA(string filepath);
    static vector<PointCloud> GeneratePointCloudFromTwitterFileWithKeywordUSA2(string filepath);
    
    static bool StorePointCloudIntoFile(string filepath, vector<PointCloud> &dataset);
    static bool StorePointCloudIntoFileWithKeyword(string filepath, map<string, PointCloud>& dataset);
    
    static vector<PointCloud> RestorePointCloudFromFile(string filepath);
    static vector<PointCloud> RestorePointCloudFromFileWithKeyword(string filepath);
//    static vector<PointCloud> RestorePointCloudFromFileWithKeyword(string filepath, vector<Point> points);
    
    static void ProcessWithKCenter(string filepath, vector<PointCloud> &dataset);
    
    static void removeNonCharacter(string input, string output);
    static void transferToLowercase(string input, string output);
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
    
    // you can remove this later
//    ofstream outfile;
//    outfile.open("/Users/lizhe/Desktop/reports/final_01/keyword_size");
    
    for(auto it = dataset_feature.begin(); it != dataset_feature.end(); it++){
//        cout << it->first << " " << it->second.pointcloud.size() << endl;
//        outfile << it->first << " " << it->second.pointcloud.size() << endl;
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

map<string, PointCloud> Dataset::GeneratePointCloudFromTwitterFileWithKeyword(string filepath){
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
    
    bool contains_only_character = true;
    
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
            contains_only_character = std::regex_match(feature, std::regex("^[A-Za-z]+$"));
            if (!contains_only_character){
                continue;
            }
            if(dataset_feature.find(feature) == dataset_feature.end()){
                PointCloud pc = PointCloud();
                pc.keyword = feature;
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
    
//    long min = 100000000;
//    long max = 0;
//    long totalpoints = 0;
//    long totalpointclouds = 0;
    
    // you can remove this later
//    ofstream outfile;
//    outfile.open("/Users/lizhe/Desktop/reports/final_01/keyword_size");
//
//    for(auto it = dataset_feature.begin(); it != dataset_feature.end(); it++){
//        //        cout << it->first << " " << it->second.pointcloud.size() << endl;
//        outfile << it->first << " " << it->second.pointcloud.size() << endl;
//        dataset.push_back(it->second);
//        if(it->second.pointcloud.size() < min){
//            min = it->second.pointcloud.size();
//        }
//        if(it->second.pointcloud.size() > max){
//            max = it->second.pointcloud.size();
//        }
//        totalpoints += it->second.pointcloud.size();
//        totalpointclouds += 1;
//    }
//    cout << ">>>>>>>>> size : " << dataset.size() << " max: "<< max << "  min: " << min << "  Average points: " << totalpoints/totalpointclouds << " totalpoints:" << totalpoints << "  totalpointclouds: " << totalpointclouds << endl;
    
    return dataset_feature;
}

vector<PointCloud> Dataset::GeneratePointCloudFromTwitterFileWithKeyword2(string filepath){
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
    
    bool contains_only_character = true;
    
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
            contains_only_character = std::regex_match(feature, std::regex("^[A-Za-z]+$"));
            if (!contains_only_character){
                continue;
            }
            if(dataset_feature.find(feature) == dataset_feature.end()){
                PointCloud pc = PointCloud();
                pc.keyword = feature;
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
        //        cout << it->first << " " << it->second.pointcloud.size() << endl;
//            outfile << it->first << " " << it->second.pointcloud.size() << endl;
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

vector<Point> Dataset::GetAllPoints(string filepath){
    vector<Point> points;
    
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
    int lines = 0;
    
    bool contains_only_character = true;
    
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
        point.keywords = set<string>(std::make_move_iterator(features.begin()), std::make_move_iterator(features.end()));
        points.push_back(point);
  
        if(lines%10000 == 0){
            cout << lines << endl;
        }
    }
    return points;
}

void Dataset::GenerateTweetPointCloudsAndAllPoints(string filepath, vector<PointCloud> &pcs, vector<Point> &ps){
    
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
    
    bool contains_only_character = true;
    int featureId = 0;
    ps.clear();
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
//            contains_only_character = std::regex_match(feature, std::regex("^[A-Za-z]+$"));
//            if (!contains_only_character){
//                continue;
//            }
            if(dataset_feature.find(feature) == dataset_feature.end()){
                PointCloud pc = PointCloud();
                pc.keyword = feature;
                pc.keywordId = featureId;
                featureId++;
                pc.dimension = 2;
                pc.pointcloud.push_back(point);
                dataset_feature.insert(pair<string, PointCloud>(feature, pc));
                point.keywordIds.insert(pc.keywordId);
            } else {
                dataset_feature[feature].pointcloud.push_back(point);
                point.keywordIds.insert(dataset_feature[feature].keywordId);
            }
        }
        ps.push_back(point);
        
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
    
    pcs.clear();
    for(auto it = dataset_feature.begin(); it != dataset_feature.end(); it++){
        pcs.push_back(it->second);
    }

}

map<string, PointCloud> Dataset::GeneratePointCloudFromTwitterFileWithKeywordUSA(string filepath){
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
    
    bool contains_only_character = true;
    
    while(getline(infile,str)){
        lines++;
        fields.clear();
        split(fields, str, boost::is_any_of("\t"));
        feature = fields[1];
        latitude_str = fields[2];
        longitude_str = fields[3];
        latitude = stod(latitude_str);
        longitude = stod(longitude_str);
        
        if(!(latitude >= 24.7433195 && latitude <= 49.3457868 && longitude >= -124.7844079 && longitude <= -66.9513812)){
            continue; // not in USA continent
        }
        
        split(features, feature, boost::is_any_of(","));
        
        Point point = Point(latitude, longitude);
        point.dimension = 2;
        for (int i = 0; i < features.size(); i++){
            feature = features[i];
            contains_only_character = std::regex_match(feature, std::regex("^[A-Za-z]+$"));
            if (!contains_only_character){
                continue;
            }
            if(dataset_feature.find(feature) == dataset_feature.end()){
                PointCloud pc = PointCloud();
                pc.keyword = feature;
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
    }
    
    return dataset_feature;
}

vector<PointCloud> Dataset::GeneratePointCloudFromTwitterFileWithKeywordUSA2(string filepath){
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
    
    bool contains_only_character = true;
    
    while(getline(infile,str)){
        lines++;
        fields.clear();
        split(fields, str, boost::is_any_of("\t"));
        feature = fields[1];
        latitude_str = fields[2];
        longitude_str = fields[3];
        latitude = stod(latitude_str);
        longitude = stod(longitude_str);
        
        if(!(latitude >= 24.7433195 && latitude <= 49.3457868 && longitude >= -124.7844079 && longitude <= -66.9513812)){
            continue; // not in USA continent
        }
        
        split(features, feature, boost::is_any_of(","));
        
        Point point = Point(latitude, longitude);
        point.dimension = 2;
        for (int i = 0; i < features.size(); i++){
            feature = features[i];
            contains_only_character = std::regex_match(feature, std::regex("^[A-Za-z]+$"));
            if (!contains_only_character){
                continue;
            }
            if(dataset_feature.find(feature) == dataset_feature.end()){
                PointCloud pc = PointCloud();
                pc.keyword = feature;
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
        //        cout << it->first << " " << it->second.pointcloud.size() << endl;
        //            outfile << it->first << " " << it->second.pointcloud.size() << endl;
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

bool Dataset::StorePointCloudIntoFileWithKeyword(string filepath, map<string, PointCloud>& dataset){
    bool isSuccess = false;
    
    ofstream outfile;
    outfile.open(filepath);
    
    for(auto it = dataset.begin(); it != dataset.end(); it++){
        outfile << "====" << endl;
        outfile << it->first << endl;
        for(int j = 0; j < it->second.pointcloud.size(); j++){
            outfile << it->second.pointcloud[j].x << " " << it->second.pointcloud[j].y << endl;
        }
    }
    isSuccess = true;
    return isSuccess;
}

vector<PointCloud> Dataset::RestorePointCloudFromFile(string filepath){
    vector<PointCloud> dataset;
    
//    // use for obervation
//    double maxx = 0, maxy =0;
//    double minx = 200, miny = 200;
    
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
            
//            if(xcor > maxx){
//                maxx = xcor;
//            }
//            if(xcor < minx){
//                minx = xcor;
//            }
//            if(ycor > maxy){
//                maxy = ycor;
//            }
//            if(ycor < miny){
//                miny = ycor;
//            }
            
            Point point = Point(xcor, ycor);
            pc.pointcloud.push_back(point);
        }
    }
    
//    cout << "x max: " << maxx << endl;
//    cout << "y max: " << maxy << endl;
//    cout << "x min: " << minx << endl;
//    cout << "y min: " << miny << endl;
    
    return dataset;
}

vector<PointCloud> Dataset::RestorePointCloudFromFileWithKeyword(string filepath){
    vector<PointCloud> dataset;
    
    //    // use for obervation
    //    double maxx = 0, maxy =0;
    //    double minx = 200, miny = 200;
    
    ifstream infile;
    infile.open(filepath);
    string str;
    vector<string> coordinates;
    double xcor, ycor;
    PointCloud pc = PointCloud();
//    PointCloud pc;
    int count = 0;
    
    bool firstFlag = true;
    bool keywordFlag = false;
    while(getline(infile, str)){
        count++;
        if(count %10000 == 0){
            cout << count <<endl;
        }
        if(str == "===="){
            if(!firstFlag){
                dataset.push_back(pc);
                pc.pointcloud.clear();
            }else {
                firstFlag = false;
            }
            keywordFlag = true;
        } else if(keywordFlag){
            pc.keyword = str;
            keywordFlag = false;
        } else {
            split(coordinates, str, boost::is_any_of(" "));
            xcor = stod(coordinates[0]);
            ycor = stod(coordinates[1]);
            
            //            if(xcor > maxx){
            //                maxx = xcor;
            //            }
            //            if(xcor < minx){
            //                minx = xcor;
            //            }
            //            if(ycor > maxy){
            //                maxy = ycor;
            //            }
            //            if(ycor < miny){
            //                miny = ycor;
            //            }
            
            Point point = Point(xcor, ycor);
            pc.pointcloud.push_back(point);
        }
    }
    
    //    cout << "x max: " << maxx << endl;
    //    cout << "y max: " << maxy << endl;
    //    cout << "x min: " << minx << endl;
    //    cout << "y min: " << miny << endl;
    
    return dataset;
}

void Dataset::ProcessWithKCenter(string filepath, vector<PointCloud> &dataset){
    ofstream outfile;
    outfile.open(filepath);
    
    for(int i = 0; i < dataset.size(); i++){
        cout << "calculating..." << i << endl;
        dataset[i].sortByKcenter();
        for(int j = 0; j < dataset[i].pointcloud.size(); j++){
            outfile << dataset[i].pointcloud[j].x << " " << dataset[i].pointcloud[j].y << endl;
        }
        outfile << "====" << endl;
    }
}

void Dataset::removeNonCharacter(string input, string output){
    
    ifstream infile;
    infile.open(input);
    
    ofstream outfile;
    outfile.open(output);
    
    vector<string> fields;
    string str;
    
    string feature;
    vector<string> features;
    string latitude_str;
    string longitude_str;

    int lines = 0;
    
    bool contains_only_character = true;
    vector<string> validKeyword;
    
    while(getline(infile,str)){
        lines++;
        fields.clear();
        split(fields, str, boost::is_any_of("\t"));
        feature = fields[1];
        latitude_str = fields[2];
        longitude_str = fields[3];
        split(features, feature, boost::is_any_of(","));
        validKeyword.clear();
        
        for (int i = 0; i < features.size(); i++){
            feature = features[i];
            contains_only_character = std::regex_match(feature, std::regex("^[A-Za-z]+$"));
            if (!contains_only_character){
                continue;
            } else{
                validKeyword.push_back(feature);
            }
        }
        
        // need to write this point to output
        if(validKeyword.size() > 0){
            outfile << fields[0] << "\t";
            
            for(int i = 0; i < validKeyword.size()-1; i++){
                outfile << validKeyword[i] << ",";
            }
            outfile << validKeyword[validKeyword.size()-1];
            outfile << "\t" << latitude_str << "\t" << longitude_str << endl;
        }
        
        if(lines%10000 == 0){
            cout << lines << endl;
        }
    }
}

// must call the above function first!!!
void Dataset::transferToLowercase(string input, string output){
    ifstream infile;
    infile.open(input);
    
    ofstream outfile;
    outfile.open(output);
    
    vector<string> fields;
    string str;
    
    string feature;
    vector<string> features;
    string latitude_str;
    string longitude_str;
    
    int lines = 0;
    
    bool contains_only_character = true;
    vector<string> validKeyword;
    
    while(getline(infile,str)){
        lines++;
        fields.clear();
        split(fields, str, boost::is_any_of("\t"));
        feature = fields[1];
        latitude_str = fields[2];
        longitude_str = fields[3];
        split(features, feature, boost::is_any_of(","));
        validKeyword.clear();
        
        for (int i = 0; i < features.size(); i++){
            feature = features[i];
            boost::algorithm::to_lower(feature);
            validKeyword.push_back(feature);
        }
        
        // need to write this point to output
        outfile << fields[0] << "\t";
        for(int i = 0; i < validKeyword.size()-1; i++){
            outfile << validKeyword[i] << ",";
        }
        outfile << validKeyword[validKeyword.size()-1];
        outfile << "\t" << latitude_str << "\t" << longitude_str << endl;
        
        if(lines%10000 == 0){
            cout << lines << endl;
        }
    }
}

#endif /* Dataset_hpp */
