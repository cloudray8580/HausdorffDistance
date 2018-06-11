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
    
    // get all points
//    static vector<Point> GetAllPoints(string filepath);
    
    // don't use this method
//    static void GenerateTweetPointCloudsAndAllPoints(string filepath, vector<PointCloud> &pcs, vector<Point> &ps);
    
    // change keyword into id
    static map<string, int> GenerateKeywordIdMapFromOriginal(string input, string output);
    static void mapKeywordIntoId(string input, string output, map<string,int> keywordIdMap); // based on original file
    static vector<Point> RestorePointFromFileWithKeywordId(string input); // based on original file with keyword Id
    static vector<PointCloud> GeneratePointCloudFromTwitterFileWithKeywordId(string filepath);
    static bool StorePointCloudIntoFileWithKeywordId(string filepath, vector<PointCloud> pcs);
    static vector<PointCloud> RestorePointCloudFromFileWithKeywordId(string filepath); // point clouds file
    
    // binary read write
    static void Binary_StoreKeywordIdDatasetToFile(string filepath, vector<PointCloud> dataset);
    static vector<PointCloud> Binary_RestoreKeywordIdDatasetFromFile(string filepath);
    static void Binary_StoreKeywordIdPointsToFile(string filepath, vector<Point> dataset);
    static vector<Point> Binary_RestoreKeywordIdPointsFromFile(string filepath);
    static void Binary_RestoreKeywordIdPointsFromFile_Seperate(string filepath, vector<Point> &points, map<int, vector<int>> &pidMap);
    
    // about USA
    static map<string, PointCloud> GeneratePointCloudFromTwitterFileWithKeywordUSA(string filepath);
    static vector<PointCloud> GeneratePointCloudFromTwitterFileWithKeywordUSA2(string filepath);
    static map<string, PointCloud> GeneratePointCloudFromTwitterFileWithKeywordNewYork(string filepath);
    static map<string, PointCloud> GeneratePointCloudFromTwitterFileWithKeywordStatenIsland(string filepath);
    static map<string, PointCloud> GeneratePointCloudFromTwitterFileWithKeywordWashington(string filepath);
    
    // store point cloud (using keyword)
    static bool StorePointCloudIntoFile(string filepath, vector<PointCloud> &dataset);
    static bool StorePointCloudIntoFileWithKeyword(string filepath, map<string, PointCloud>& dataset);
    
    // restore point cloud (using keyword)
    static vector<PointCloud> RestorePointCloudFromFile(string filepath);
    static vector<PointCloud> RestorePointCloudFromFileWithKeyword(string filepath);
    
    // generate keyword-id map
    static map<string, int> GenerateKeywordIdMap(string input, string output); // using point clouds file
    static map<string, int> RestoreKeywordIdMap(string input);
//    static vector<PointCloud> RestorePointCloudFromFileWithKeyword(string filepath, vector<Point> points);
    
    // generate kcenters
    static void ProcessWithKCenter(string filepath, vector<PointCloud> &dataset);
    
    // preprocessing
    static void removeNonCharacter(string input, string output);
    static void transferToLowercase(string input, string output);
    
    // sampling
    static void randomSampling(string input, string output, int percentage);
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

//vector<Point> Dataset::GetAllPoints(string filepath){
//    vector<Point> points;
//
//    ifstream infile;
//    infile.open(filepath);
//    vector<string> fields;
//    string str;
//
//    string coordinate;
//    string feature;
//    vector<string> features;
//    string latitude_str;
//    string longitude_str;
//    double latitude;
//    double longitude;
//    int lines = 0;
//
//    bool contains_only_character = true;
//
//    while(getline(infile,str)){
//        lines++;
//        fields.clear();
//        split(fields, str, boost::is_any_of("\t"));
//        feature = fields[1];
//        latitude_str = fields[2];
//        longitude_str = fields[3];
//        latitude = stod(latitude_str);
//        longitude = stod(longitude_str);
//        split(features, feature, boost::is_any_of(","));
//
//        Point point = Point(latitude, longitude);
//        point.dimension = 2;
//        point.keywords = set<string>(std::make_move_iterator(features.begin()), std::make_move_iterator(features.end()));
//        points.push_back(point);
//
//        if(lines%10000 == 0){
//            cout << lines << endl;
//        }
//    }
//    return points;
//}

//void Dataset::GenerateTweetPointCloudsAndAllPoints(string filepath, vector<PointCloud> &pcs, vector<Point> &ps){
//
//    ifstream infile;
//    infile.open(filepath);
//    vector<string> fields;
//    string str;
//
//    string coordinate;
//    string feature;
//    vector<string> features;
//    string latitude_str;
//    string longitude_str;
//    double latitude;
//    double longitude;
//    map<string, PointCloud> dataset_feature;
//    int lines = 0;
//
//    bool contains_only_character = true;
//    int featureId = 0;
//    ps.clear();
//    while(getline(infile,str)){
//        lines++;
//        fields.clear();
//        split(fields, str, boost::is_any_of("\t"));
//        feature = fields[1];
//        latitude_str = fields[2];
//        longitude_str = fields[3];
//        latitude = stod(latitude_str);
//        longitude = stod(longitude_str);
//        split(features, feature, boost::is_any_of(","));
//
//        Point point = Point(latitude, longitude);
//        point.dimension = 2;
//        for (int i = 0; i < features.size(); i++){
//            feature = features[i];
////            contains_only_character = std::regex_match(feature, std::regex("^[A-Za-z]+$"));
////            if (!contains_only_character){
////                continue;
////            }
//            if(dataset_feature.find(feature) == dataset_feature.end()){
//                PointCloud pc = PointCloud();
//                pc.keyword = feature;
//                pc.keywordId = featureId;
//                featureId++;
//                pc.dimension = 2;
//                pc.pointcloud.push_back(point);
//                dataset_feature.insert(pair<string, PointCloud>(feature, pc));
////                point.keywordIds.insert(pc.keywordId);
//                point.keywordIds.push_back(pc.keywordId);
//            } else {
//                dataset_feature[feature].pointcloud.push_back(point);
////                point.keywordIds.insert(dataset_feature[feature].keywordId);
//                 point.keywordIds.push_back(dataset_feature[feature].keywordId);
//            }
//        }
//        ps.push_back(point);
//
//        if(lines%10000 == 0){
//            cout << lines << endl;
//        }
//    }
//
//    long min = 100000000;
//    long max = 0;
//    long totalpoints = 0;
//    long totalpointclouds = 0;
//
//    pcs.clear();
//    for(auto it = dataset_feature.begin(); it != dataset_feature.end(); it++){
//        pcs.push_back(it->second);
//    }
//
//}

map<string, int> Dataset::GenerateKeywordIdMapFromOriginal(string input, string output){
    
    map<string, int> keywordIdMap;
    vector<PointCloud> dataset;
    
    ifstream infile;
    infile.open(input);
    vector<string> fields;
    string str;
    string feature;
    vector<string> features;

    map<string, PointCloud> dataset_feature;
    int lines = 0;
    
    bool contains_only_character = true;
    
    while(getline(infile,str)){
        lines++;
        fields.clear();
        split(fields, str, boost::is_any_of("\t"));
        feature = fields[1];
        split(features, feature, boost::is_any_of(","));
        for (int i = 0; i < features.size(); i++){
            feature = features[i];
            if(keywordIdMap.find(feature) == keywordIdMap.end()){
                keywordIdMap.insert(pair<string, int>(feature, 0));
            }
        }
        if(lines%10000 == 0){
            cout << lines << endl;
        }
    }
    
    int keywordId = 0;
    ofstream outfile;
    outfile.open(output);
    for(auto it = keywordIdMap.begin(); it != keywordIdMap.end(); it++){
        it->second = keywordId;
        outfile << it->first << " " << it->second << endl;
        keywordId++;
    }
    return keywordIdMap;
}

void Dataset::mapKeywordIntoId(string input, string output, map<string,int> keywordIdMap){
    
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
    vector<int> keywordId;
    
    while(getline(infile,str)){
        lines++;
        fields.clear();
        split(fields, str, boost::is_any_of("\t"));
        feature = fields[1];
        latitude_str = fields[2];
        longitude_str = fields[3];
        split(features, feature, boost::is_any_of(","));
        keywordId.clear();
        for (int i = 0; i < features.size(); i++){
            feature = features[i];
            keywordId.push_back(keywordIdMap[feature]);
        }
        // need to write this point to output
        outfile << fields[0] << "\t";
        for(int i = 0; i < keywordId.size()-1; i++){
            outfile << keywordId[i] << ",";
        }
        outfile << keywordId[keywordId.size()-1];
        outfile << "\t" << latitude_str << "\t" << longitude_str << endl;
        
        if(lines%10000 == 0){
            cout << lines << endl;
        }
    }
}

vector<Point> Dataset::RestorePointFromFileWithKeywordId(string input){
    vector<Point> points;

    ifstream infile;
    infile.open(input);
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
    vector<int> featureIds;

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
        featureIds.clear();
        for (int i = 0; i < features.size(); i++){
            feature = features[i];
            featureIds.push_back(stoi(feature));
        }
        Point point = Point(latitude, longitude);
        point.dimension = 2;
        point.keywordIds = featureIds;
        point.keywordsize = featureIds.size();
//        point.keywordIds = set<int>(std::make_move_iterator(featureIds.begin()), std::make_move_iterator(featureIds.end()));
        points.push_back(point);

        if(lines%10000 == 0){
            cout << lines << endl;
        }
    }
    return points;
}

vector<PointCloud> Dataset::GeneratePointCloudFromTwitterFileWithKeywordId(string filepath){
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
    map<int, PointCloud> dataset_feature;
    int lines = 0;
    
    bool contains_only_character = true;
    int featureId;
    
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
            featureId = stoi(feature);
            if(dataset_feature.find(featureId) == dataset_feature.end()){
                PointCloud pc = PointCloud();
                pc.keywordId = featureId;
                pc.dimension = 2;
                pc.pointcloud.push_back(point);
                dataset_feature.insert(pair<int, PointCloud>(featureId, pc));
            } else {
                dataset_feature[featureId].pointcloud.push_back(point);
            }
        }
        
        if(lines%10000 == 0){
            cout << lines << endl;
        }
    }
    
    for(auto it = dataset_feature.begin(); it != dataset_feature.end(); it++){
        it->second.size = it->second.pointcloud.size();
        dataset.push_back(it->second);
    }
    
    return dataset;
}

bool Dataset::StorePointCloudIntoFileWithKeywordId(string filepath, vector<PointCloud> pcs){
    bool isSuccess = false;
    
    ofstream outfile;
    outfile.open(filepath);
    
    for(int i = 0; i <= pcs.size(); i++){
        outfile << "====" << endl;
        outfile << pcs[i].keywordId << endl;
        for(int j = 0; j < pcs[i].pointcloud.size(); j++){
            outfile << pcs[i].pointcloud[j].x << " " << pcs[i].pointcloud[j].y << endl;
        }
    }
    isSuccess = true;
    return isSuccess;
}

vector<PointCloud> Dataset::RestorePointCloudFromFileWithKeywordId(string filepath){
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
                pc.size = pc.pointcloud.size();
                dataset.push_back(pc);
                pc.pointcloud.clear();
            }else {
                firstFlag = false;
            }
            keywordFlag = true;
        } else if(keywordFlag){
            pc.keywordId = stoi(str);
            keywordFlag = false;
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

void Dataset::Binary_StoreKeywordIdDatasetToFile(string filepath, vector<PointCloud> dataset){
    ofstream outfile;
    outfile.open(filepath, ios::binary);
    for(int i = 0; i < dataset.size(); i++){
        outfile.write((char*)&dataset[i].keywordId, sizeof(int));
        outfile.write((char*)&dataset[i].size, sizeof(int));
        for(int j = 0; j < dataset[i].pointcloud.size(); j++){
            outfile.write((char*)&dataset[i].pointcloud[j].x, sizeof(double));
            outfile.write((char*)&dataset[i].pointcloud[j].y, sizeof(double));
        }
    }
}

vector<PointCloud> Dataset::Binary_RestoreKeywordIdDatasetFromFile(string filepath){
    
    vector<PointCloud> dataset;
    
    ifstream infile;
    infile.open(filepath, ios::binary);
    PointCloud pc = PointCloud();
    Point p(0, 0);
    int count = 0;
    while(true){
        infile.read((char*)&pc.keywordId, sizeof(int));
        infile.read((char*)&pc.size, sizeof(int));
        pc.pointcloud.clear();
        for(int i = 0; i < pc.size; i++){
            pc.pointcloud.push_back(p);
            infile.read((char*)&pc.pointcloud[i].x, sizeof(double));
            infile.read((char*)&pc.pointcloud[i].y, sizeof(double));
        }
        dataset.push_back(pc);
        if(!infile){
            break;
        }
        count++;
        if(count % 10000 == 0){
            cout << count << endl;
        }
    }
    return dataset;
}

void Dataset::Binary_StoreKeywordIdPointsToFile(string filepath, vector<Point> dataset){
    ofstream outfile;
    outfile.open(filepath, ios::binary);
    for(int i = 0; i < dataset.size(); i++){
        outfile.write((char*)&dataset[i].x, sizeof(double));
        outfile.write((char*)&dataset[i].y, sizeof(double));
        outfile.write((char*)&dataset[i].keywordsize, sizeof(int));
        for(int j = 0; j < dataset[i].keywordsize; j++){
            outfile.write((char*)&dataset[i].keywordIds[j], sizeof(int));
        }
    }
}

vector<Point> Dataset::Binary_RestoreKeywordIdPointsFromFile(string filepath){
    vector<Point> dataset;

    ifstream infile;
    infile.open(filepath, ios::binary);
    Point p(0, 0);
    int count = 0;
    while(true){
        if(!infile){
            break;
        }
        infile.read((char*)&p.x, sizeof(double));
        infile.read((char*)&p.y, sizeof(double));
        infile.read((char*)&p.keywordsize, sizeof(int));
        p.keywordIds.clear();
        for(int i = 0; i < p.keywordsize; i++){
            p.keywordIds.push_back(0);
            infile.read((char*)&p.keywordIds[i], sizeof(int));
        }
        dataset.push_back(p);
        count++;
        if(count % 10000 == 0){
            cout << count << endl;
        }
    }
    return dataset;
}

void Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate(string filepath, vector<Point> &points, map<int, vector<int>> &pidMap){
    
    ifstream infile;
    infile.open(filepath, ios::binary);
    Point p(0, 0);
    int count = 0;
    vector<int> keywordIds;
    while(true){
        if(!infile){
            break;
        }
        infile.read((char*)&p.x, sizeof(double));
        infile.read((char*)&p.y, sizeof(double));
        infile.read((char*)&p.keywordsize, sizeof(int));
        p.pid = count;
        keywordIds.clear();
//        if(count == 472603){
//            cout << "here" << endl;
//        }
        for(int i = 0; i < p.keywordsize; i++){
            keywordIds.push_back(0);
            infile.read((char*)&keywordIds[i], sizeof(int));
        }
        pidMap.insert(pair<int, vector<int>>(count,keywordIds));
        points.push_back(p);
        count++;
        if(count % 10000 == 0){
            cout << count << endl;
        }
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
//        if(!(latitude >= 40.49663297753 && latitude <= 40.91250809 && longitude >= -74.254209408 && longitude <= -73.7373388529)){
//            continue; // not in New York
//        }
        
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

// the South West Point of Staten Island, the North point of Bronx, the East point of Queens
map<string, PointCloud> Dataset::GeneratePointCloudFromTwitterFileWithKeywordNewYork(string filepath){
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
        
        if(!(latitude >= 40.49663297753 && latitude <= 40.91250809 && longitude >= -74.254209408 && longitude <= -73.7373388529)){
            continue; // not in New York
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

map<string, PointCloud> Dataset::GeneratePointCloudFromTwitterFileWithKeywordWashington(string filepath){
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
        
//        if(!(latitude >= 38.7995439804529 && latitude <= 38.98634172844672 && longitude >= -77.19166513763514
//&& longitude <= -76.88233133636561)){
//            continue; // not in New York
//        }
        
        if(!(latitude >= 38.80 && latitude <= 38.88 && longitude >= -77.155
             && longitude <= -77.101)){
            continue; // not in New York
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

map<string, PointCloud> Dataset::GeneratePointCloudFromTwitterFileWithKeywordStatenIsland(string filepath){
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
        
        if(!(latitude >= 40.49710982749934 && latitude <= 40.64310576839339 && longitude >= -74.25426101760928 && longitude <= -74.05978375641905)){
            continue; // not in New York
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

map<string, int> Dataset::GenerateKeywordIdMap(string input, string output){
    
    map<string, int> keywordIdMap;
    
    ifstream infile;
    infile.open(input);
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
            keywordFlag = true;
        } else if(keywordFlag){
            pc.keyword = str;
            keywordIdMap.insert(pair<string, int>(str,0));
            keywordFlag = false;
        } else {
            continue;
        }
    }
    
    ofstream outfile;
    outfile.open(output);
    int keywordId = 0;
    for(auto it = keywordIdMap.begin(); it != keywordIdMap.end(); it++){
        it->second = keywordId;
        outfile << it->first << " " << it->second << endl;
        keywordId++;
    }
    return keywordIdMap;
}

map<string, int> Dataset::RestoreKeywordIdMap(string input){
    
    map<string, int> keywordIdMap;
    
    ifstream infile;
    infile.open(input);
    string str;
    vector<string> keywordAndId;
    string keyword;
    int id;
    
    int count = 0;
    
    bool firstFlag = true;
    bool keywordFlag = false;
    while(getline(infile, str)){
        count++;
        if(count %10000 == 0){
            cout << count <<endl;
        }
        split(keywordAndId, str, boost::is_any_of(" "));
        keyword = keywordAndId[0];
        id = stoi(keywordAndId[1]);
        keywordIdMap.insert(pair<string, int>(keyword, id));
    }
    return keywordIdMap;
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

void Dataset::randomSampling(string input, string output, int percentage){
    ifstream infile;
    infile.open(input);
    
    ofstream outfile;
    outfile.open(output);
    
    string str;
    int lines = 0;
    
    bool contains_only_character = true;
    srand((unsigned)time(NULL));
    int possibility = 0;
    
    while(getline(infile,str)){
        lines++;
        possibility = rand()%100;
        
        if(possibility <= percentage){
            outfile << str << endl;
        }
        
        if(lines%10000 == 0){
            cout << lines << endl;
        }
    }
}

#endif /* Dataset_hpp */
