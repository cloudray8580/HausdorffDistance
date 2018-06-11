//
//  FinalExperiment.hpp
//  HausdorffDistance
//
//  Created by 李喆 on 2018/5/31.
//  Copyright © 2018年 李喆. All rights reserved.
//

#ifndef FinalExperiment_hpp
#define FinalExperiment_hpp

#include <stdio.h>
#include <sstream>
#include "Comparison.hpp"
using namespace std;

void prepareQuerySet(){
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary"); // world - character - lowercase - binary - keywordId
    
    int typeQ1 = 0; // [1 ~ 9)
    int typeQ2 = 0; // [10 - 99)
    int typeQ3 = 0; // [100 ~ 999)
    int typeQ4 = 0; // [1000 ~ 9999)
    int typeQ5 = 0; // [10000 ~ 99999)
    
    stringstream ss;
    
    for(int i = 0; i < dataset.size(); i++){
        if(dataset[i].pointcloud.size() >= 1 && dataset[i].pointcloud.size() < 9 && typeQ1 < 10){
            ss.str("");
            ss << "/Users/lizhe/Desktop/dataset/querySet/Q1-" << typeQ1 << ".pts";
            dataset[i].storeToFile(ss.str());
            typeQ1++;
            continue;
        }
        
        if(dataset[i].pointcloud.size() >= 10 && dataset[i].pointcloud.size() < 99 && typeQ2 < 10){
            ss.str("");
            ss << "/Users/lizhe/Desktop/dataset/querySet/Q2-" << typeQ2 << ".pts";
            dataset[i].storeToFile(ss.str());
            typeQ2++;
            continue;
        }
        
        if(dataset[i].pointcloud.size() >= 100 && dataset[i].pointcloud.size() < 999 && typeQ3 < 10){
            ss.str("");
            ss << "/Users/lizhe/Desktop/dataset/querySet/Q3-" << typeQ3 << ".pts";
            dataset[i].storeToFile(ss.str());
            typeQ3++;
            continue;
        }
        
        if(dataset[i].pointcloud.size() >= 1000 && dataset[i].pointcloud.size() <= 9999 && typeQ4 < 10){
            ss.str("");
            ss << "/Users/lizhe/Desktop/dataset/querySet/Q4-" << typeQ4 << ".pts";
            dataset[i].storeToFile(ss.str());
            typeQ4++;
            continue;
        }
        
        if(dataset[i].pointcloud.size() >= 10000 && dataset[i].pointcloud.size() <= 99999 && typeQ5 < 10){
            ss.str("");
            ss << "/Users/lizhe/Desktop/dataset/querySet/Q5-" << typeQ5 << ".pts";
            dataset[i].storeToFile(ss.str());
            typeQ5++;
            continue;
        }
    }
}

void experiment_1_preparement(){
    
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary");
    KNNSearch knn(dataset, points);

    // the max capacity of MBR is 10 all
    knn.generateMBRsForDataset("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-2", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-2", dataset, 2);
    knn.generateMBRsForDataset("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-5", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-5", dataset, 5);
    knn.generateMBRsForDataset("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-10", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-10", dataset, 10);
    knn.generateMBRsForDataset("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-20", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-20", dataset, 20);
    knn.generateMBRsForDataset("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-50", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-50", dataset, 50);
    knn.generateMBRsForDataset("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-100", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-100", dataset, 100);
    
//    knn.associateMBRs("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-2", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-2");
}


void experiment_1(string filepath, int NumberOfMBRs){
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary");
    KNNSearch knn(dataset, points);
    
    stringstream ss1, ss2;
    ss1 << "/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-" << NumberOfMBRs;
    ss2 << "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-" << NumberOfMBRs;
    knn.associateMBRs(ss1.str(), ss2.str());
    
    stringstream ss;
    ofstream outfile;
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q1-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_GIS2011(pc, 10, NumberOfMBRs, filepath);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q2-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_GIS2011(pc, 10, NumberOfMBRs, filepath);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q3-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_GIS2011(pc, 10, NumberOfMBRs, filepath);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q4-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_GIS2011(pc, 10, NumberOfMBRs, filepath);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q5-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_GIS2011(pc, 10, NumberOfMBRs, filepath);
    }
}

void experiment_2(string filepath, double kcenterPercent){
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary");
    KNNSearch knn(dataset, points);
    
    stringstream ss;
    ofstream outfile;
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();

    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q1-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_PAMI2015_Pruning_KCenter(pc, 10, kcenterPercent, filepath);
    }

    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();

    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q2-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_PAMI2015_Pruning_KCenter(pc, 10, kcenterPercent, filepath);
    }

    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();

    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q3-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_PAMI2015_Pruning_KCenter(pc, 10, kcenterPercent, filepath);
    }

    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();

    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q4-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_PAMI2015_Pruning_KCenter(pc, 10, kcenterPercent, filepath);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q5-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_PAMI2015_Pruning_KCenter(pc, 10, kcenterPercent, filepath);
    }
}

void experiment_3(string filepath, double UBP){
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary");
    KNNSearch knn(dataset, points);
    knn.buildRtreeForAllPoints();
    
    stringstream ss;
    ofstream outfile;
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q1-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepath, UBP, 1);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q2-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepath, UBP, 1);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q3-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepath, UBP, 1);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q4-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepath, UBP, 1);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q5-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepath, UBP, 1);
    }
}

void experiment_3_2(string filepath, double UBP){
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary");
    KNNSearch knn(dataset, points);
    knn.buildRtreeForAllPoints();
    knn.generateKeywordIdMap();
    
    stringstream ss;
    ofstream outfile;
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q1-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient(pc, 10, pidMap, filepath, UBP, 1);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q2-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient(pc, 10, pidMap, filepath, UBP, 1);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q3-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient(pc, 10, pidMap, filepath, UBP, 1);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q4-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient(pc, 10, pidMap, filepath, UBP, 1);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q5-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient(pc, 10, pidMap, filepath, UBP, 1);
    }
}

void experiment_4(string filepath, int PNum){
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary");
    KNNSearch knn(dataset, points);
    knn.buildRtreeForAllPoints();
    
    stringstream ss;
    ofstream outfile;
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q1-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepath, 0.1, PNum);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q2-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepath, 0.1, PNum);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q3-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepath, 0.1, PNum);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q4-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepath, 0.1, PNum);
    }
    
    outfile.open(filepath, ofstream::app);
    outfile << endl;
    outfile << endl;
    outfile.close();
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q5-" << i << ".pts";
        PointCloud pc(ss.str());
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepath, 0.1, PNum);
    }
}

void experiment_5(string filepaths[], int k){
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary");
    KNNSearch knn(dataset, points);
    knn.buildRtreeForAllPoints();
    knn.generateKeywordIdMap();
//    knn.associateMBRs("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-20", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-20");
    
    
    stringstream ss;
    ofstream outfile;
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q1-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q2-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q3-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q4-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q5-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
}

void experiment_6_preparement_10_Percent(){
    
    Dataset::randomSampling("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase", "/Users/lizhe/Desktop/dataset/keyword-10Percent", 10);
    map<string, int> keywordIdMap = Dataset::GenerateKeywordIdMapFromOriginal("/Users/lizhe/Desktop/dataset/keyword-10Percent", "/Users/lizhe/Desktop/dataset/keywordIdMap-10Percent");
    Dataset::mapKeywordIntoId("/Users/lizhe/Desktop/dataset/keyword-10Percent", "/Users/lizhe/Desktop/dataset/original-10Percent", keywordIdMap);
    vector<Point> points = Dataset::RestorePointFromFileWithKeywordId("/Users/lizhe/Desktop/dataset/original-10Percent");
    Dataset::Binary_StoreKeywordIdPointsToFile("/Users/lizhe/Desktop/dataset/points-binary-10Percent", points);
    vector<PointCloud> pointclouds = Dataset::GeneratePointCloudFromTwitterFileWithKeywordId("/Users/lizhe/Desktop/dataset/original-10Percent");
    Dataset::StorePointCloudIntoFileWithKeywordId("/Users/lizhe/Desktop/dataset/pointclouds-10Percent", pointclouds);
    Dataset::Binary_StoreKeywordIdDatasetToFile("/Users/lizhe/Desktop/dataset/pointclouds-binary-10Percent", pointclouds);
}

void experiment_6_preparement_20_Percent(){
    
    Dataset::randomSampling("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase", "/Users/lizhe/Desktop/dataset/keyword-20Percent", 20);
    map<string, int> keywordIdMap = Dataset::GenerateKeywordIdMapFromOriginal("/Users/lizhe/Desktop/dataset/keyword-20Percent", "/Users/lizhe/Desktop/dataset/keywordIdMap-20Percent");
    Dataset::mapKeywordIntoId("/Users/lizhe/Desktop/dataset/keyword-20Percent", "/Users/lizhe/Desktop/dataset/original-20Percent", keywordIdMap);
    vector<Point> points = Dataset::RestorePointFromFileWithKeywordId("/Users/lizhe/Desktop/dataset/original-20Percent");
    Dataset::Binary_StoreKeywordIdPointsToFile("/Users/lizhe/Desktop/dataset/points-binary-20Percent", points);
    vector<PointCloud> pointclouds = Dataset::GeneratePointCloudFromTwitterFileWithKeywordId("/Users/lizhe/Desktop/dataset/original-20Percent");
    Dataset::StorePointCloudIntoFileWithKeywordId("/Users/lizhe/Desktop/dataset/pointclouds-20Percent", pointclouds);
    Dataset::Binary_StoreKeywordIdDatasetToFile("/Users/lizhe/Desktop/dataset/pointclouds-binary-20Percent", pointclouds);
}

void experiment_6_preparement_50_Percent(){
    
    Dataset::randomSampling("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase", "/Users/lizhe/Desktop/dataset/keyword-50Percent", 50);
    map<string, int> keywordIdMap = Dataset::GenerateKeywordIdMapFromOriginal("/Users/lizhe/Desktop/dataset/keyword-50Percent", "/Users/lizhe/Desktop/dataset/keywordIdMap-50Percent");
    Dataset::mapKeywordIntoId("/Users/lizhe/Desktop/dataset/keyword-50Percent", "/Users/lizhe/Desktop/dataset/original-50Percent", keywordIdMap);
    vector<Point> points = Dataset::RestorePointFromFileWithKeywordId("/Users/lizhe/Desktop/dataset/original-50Percent");
    Dataset::Binary_StoreKeywordIdPointsToFile("/Users/lizhe/Desktop/dataset/points-binary-50Percent", points);
    vector<PointCloud> pointclouds = Dataset::GeneratePointCloudFromTwitterFileWithKeywordId("/Users/lizhe/Desktop/dataset/original-50Percent");
    Dataset::StorePointCloudIntoFileWithKeywordId("/Users/lizhe/Desktop/dataset/pointclouds-50Percent", pointclouds);
    Dataset::Binary_StoreKeywordIdDatasetToFile("/Users/lizhe/Desktop/dataset/pointclouds-binary-50Percent", pointclouds);
}

void experiment_6_preparement_10_Percent_MBR(){
    
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary-10Percent", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary-10Percent");
    KNNSearch knn(dataset, points);
    knn.generateMBRsForDataset("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-20-10Percent", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-20-10Percent", dataset, 20);
    knn.generateMBRsForDataset("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-10-10Percent", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-10-10Percent", dataset, 10);
}

void experiment_6_preparement_20_Percent_MBR(){
    
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary-20Percent", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary-20Percent");
    KNNSearch knn(dataset, points);
    knn.generateMBRsForDataset("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-20-20Percent", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-20-20Percent", dataset, 20);
    knn.generateMBRsForDataset("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-10-20Percent", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-10-20Percent", dataset, 10);
}

void experiment_6_preparement_50_Percent_MBR(){
    
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary-50Percent", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary-50Percent");
    KNNSearch knn(dataset, points);
    knn.generateMBRsForDataset("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-20-50Percent", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-20-50Percent", dataset, 20);
    knn.generateMBRsForDataset("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-10-50Percent", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-10-50Percent", dataset, 10);
}


void experiment_6_100(string filepaths[], int k){
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary");
    KNNSearch knn(dataset, points);
    knn.buildRtreeForAllPoints();
    knn.generateKeywordIdMap();
    knn.associateMBRs("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-20", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-20");
    
    
    stringstream ss;
    ofstream outfile;
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q1-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q2-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q3-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q4-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q5-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
}

void experiment_6_50(string filepaths[], int k){
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary-50Percent", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary-50Percent");
    KNNSearch knn(dataset, points);
    knn.buildRtreeForAllPoints();
    knn.generateKeywordIdMap();
//    knn.associateMBRs("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-20-50Percent", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-20-50Percent");
    
    
    stringstream ss;
    ofstream outfile;
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }

    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q1-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }

    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }

    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q2-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }

    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }

    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q3-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }

    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }

    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q4-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }

    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q5-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
}

void experiment_6_20(string filepaths[], int k){
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary-20Percent", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary-20Percent");
    KNNSearch knn(dataset, points);
    knn.buildRtreeForAllPoints();
    knn.generateKeywordIdMap();
//    knn.associateMBRs("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-20-20Percent", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-20-20Percent");
    
    
    stringstream ss;
    ofstream outfile;
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q1-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q2-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q3-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q4-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q5-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
}

void experiment_6_10(string filepaths[], int k){
    vector<Point> points;
    map<int, vector<int>> pidMap;
    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary-10Percent", points, pidMap);
    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary-10Percent");
    KNNSearch knn(dataset, points);
    knn.buildRtreeForAllPoints();
    knn.generateKeywordIdMap();
//    knn.associateMBRs("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-20-10Percent", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-20-10Percent");
    
    
    stringstream ss;
    ofstream outfile;
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q1-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q2-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q3-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q4-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
    
    for(int i = 0; i < 5; i++){
        outfile.open(filepaths[i], ofstream::app);
        outfile << endl;
        outfile << endl;
        outfile.close();
    }
    
    for(int i = 0; i < 10; i++){
        ss.str("");
        ss << "/Users/lizhe/Desktop/dataset/querySet/Q5-" << i << ".pts";
        PointCloud pc(ss.str());
//        knn.KNN_GIS2011(pc, k, 20, filepaths[0]);
//        knn.KNN_PAMI2015_Pruning_KCenter(pc, k, 0.05, filepaths[1]);
//        knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, k, filepaths[2], 100000, 100000);
        knn.KNNSearch::KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, k, pidMap, filepaths[3], 0.1, 2);
        knn.KNN_UsingPoint_Efficient_KeywordLB(pc, k, pidMap, filepaths[4], 0.1, 2);
    }
}
#endif /* FinalExperiment_hpp */
