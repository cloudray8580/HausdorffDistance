//
//  main.cpp
//  HausdorffDistance
//
//  Created by 李喆 on 2018/1/3.
//  Copyright © 2018年 李喆. All rights reserved.
//

#include <iostream>
#include "Comparison.hpp"
#include "AnalyseTask.hpp"
#include "Util.hpp"
#include "HilbertCurve.hpp"
#include "BoostRTreeSetting.hpp"
#include "FinalExperiment.hpp"
using namespace std;

bool cmp_hilbert(pair<Point, int>& hilbertpoint1, pair<Point, int>& hilbertpoint2){
    return hilbertpoint1.second < hilbertpoint2.second;
}

int main(int argc, const char * argv[]) {
    
//    map<string, int> keywordIdMap = Dataset::GenerateKeywordIdMapFromOriginal("/Users/lizhe/Downloads/ICDE15data/Tweets-Character", "/Users/lizhe/Desktop/dataset/keywordIdMap-TweetOriginal");
//    Dataset::mapKeywordIntoId("/Users/lizhe/Downloads/ICDE15data/Tweets-Character", "/Users/lizhe/Desktop/dataset/TweetOriginal", keywordIdMap);
//    vector<Point> points = Dataset::RestorePointFromFileWithKeywordId("/Users/lizhe/Desktop/dataset/TweetOriginal");
//    Dataset::Binary_StoreKeywordIdPointsToFile("/Users/lizhe/Desktop/dataset/points-binary-TweetOriginal", points);
//    vector<PointCloud> pointclouds = Dataset::GeneratePointCloudFromTwitterFileWithKeywordId("/Users/lizhe/Desktop/dataset/TweetOriginal");
//    Dataset::StorePointCloudIntoFileWithKeywordId("/Users/lizhe/Desktop/dataset/pointclouds-TweetOriginal", pointclouds);
//    Dataset::Binary_StoreKeywordIdDatasetToFile("/Users/lizhe/Desktop/dataset/pointclouds-binary-TweetOriginal", pointclouds);
    
    
//    experiment_6_preparement_10_Percent();
//    experiment_6_preparement_20_Percent();
//    experiment_6_preparement_50_Percent();
//    experiment_6_preparement_10_Percent_MBR();
//    experiment_6_preparement_20_Percent_MBR();
//    experiment_6_preparement_50_Percent_MBR();
    
//    vector<Point> points;
//    map<int, vector<int>> pidMap;
//    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary", points, pidMap);
//    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary");
////    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary-TweetOriginal", points, pidMap);
////    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary-TweetOriginal");
//    KNNSearch knn(dataset, points);
//    knn.buildRtreeForAllPoints();
//    knn.generateKeywordIdMap();
//    knn.generateMBRsForDataset("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-20-TweetOriginal", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-20-TweetOriginal", dataset, 20);
//    knn.associateMBRs("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-20", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-20");
//    knn.associateMBRs("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-20-TweetOriginal", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-20-TweetOriginal");
    
//    PointCloud pc("/Users/lizhe/Desktop/dataset/querySet/Q5-9.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc, 10, 0.05, "/Users/lizhe/Desktop/temp1");
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, 10, "/Users/lizhe/Desktop/temp2", 100000, 100000);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, "/Users/lizhe/Desktop/temp3", 0.1, 2);
//    knn.KNN_Kcenter_KCUB_PBKC(pc, 10, pidMap, "/Users/lizhe/Desktop/temp4");

//    PointCloud pc1("/Users/lizhe/Desktop/dataset/querySet/Q4-0.pts");
//    PointCloud pc2("/Users/lizhe/Desktop/dataset/querySet/Q5-4.pts");
//    
////    pc1.randomize();
//    pc2.randomize();
//    pc1.sortByKcenter(0.01);
//    ExactHausdorff::PAMI2015(pc1, pc2);
//    pc1.sortByKcenter(0.002);
//    ExactHausdorff::PAMI2015(pc1, pc2);
//    pc1.sortByKcenter(0.005);
//    ExactHausdorff::PAMI2015(pc1, pc2);
//    pc1.sortByKcenter(0.01);
//    ExactHausdorff::PAMI2015(pc1, pc2);
//    pc1.sortByKcenter(0.02);
//    ExactHausdorff::PAMI2015(pc1, pc2);
//    pc1.sortByKcenter(0.05);
//    ExactHausdorff::PAMI2015(pc1, pc2);
//    pc1.sortByKcenter(0.1);
//    ExactHausdorff::PAMI2015(pc1, pc2);
//    pc1.sortByKcenter(0.2);
//    ExactHausdorff::PAMI2015(pc1, pc2);
    
    
    
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, "/Users/lizhe/Desktop/temp-beta", 0.01, 1);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, "/Users/lizhe/Desktop/temp-beta", 0.02, 1);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, "/Users/lizhe/Desktop/temp-beta", 0.05, 1);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, "/Users/lizhe/Desktop/temp-beta", 0.1, 1);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, "/Users/lizhe/Desktop/temp-beta", 0.2, 1);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, "/Users/lizhe/Desktop/temp-beta", 0.5, 1);
    
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, "/Users/lizhe/Desktop/temp-gamma", 0.1, 1);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, "/Users/lizhe/Desktop/temp-gamma", 0.1, 2);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, "/Users/lizhe/Desktop/temp-gamma", 0.1, 3);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, "/Users/lizhe/Desktop/temp-gamma", 0.1, 4);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, "/Users/lizhe/Desktop/temp-gamma", 0.1, 5);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, "/Users/lizhe/Desktop/temp-gamma", 0.1, 10);
    
    
//    string filepaths[5];
//    filepaths[0] = "/Users/lizhe/Desktop/reports/final_experiment/6_10_GIS2011.csv";
//    filepaths[1] = "/Users/lizhe/Desktop/reports/final_experiment/6_10_KCenter.csv";
//    filepaths[2] = "/Users/lizhe/Desktop/reports/final_experiment/6_10_KCUB.csv";
//    filepaths[3] = "/Users/lizhe/Desktop/reports/final_experiment/6_10_PointBased.csv";
//    filepaths[4] = "/Users/lizhe/Desktop/reports/final_experiment/6_10_PointBasedKcenter.csv";
//    knn.KNN_GIS2011(pc, 10, 20, filepaths[0]);
//    knn.KNN_PAMI2015_Pruning_KCenter(pc, 10, 0.05, filepaths[1]);
//    knn.KNN_PAMI2015_Pruning_KCenter(pc, 10, 0.05, filepaths[1]);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, 10, filepaths[2], 100000, 100000);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepaths[4], 0.1, 1);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepaths[4], 0.2, 1);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepaths[4], 0.3, 1);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepaths[4], 0.4, 1);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepaths[4], 0.5, 1);
    
//    knn.KNN_UsingPoint_Efficient_KeywordLB_WithoutKcenter(pc, 1, pidMap, filepaths[3], 0.1, 2);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepaths[4], 0.5, 2);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepaths[4], 0.5, 3);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepaths[4], 0.6, 1);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepaths[4], 0.7, 1);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepaths[4], 0.8, 1);
//    knn.KNN_UsingPoint_Efficient_KeywordLB(pc, 10, pidMap, filepaths[4], 0.9, 1);
    
    
//    vector<Point> points;
//    map<int, vector<int>> pidMap;
//    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary", points, pidMap);
//    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary");
//    KNNSearch knn(dataset, points);
//    knn.associateMBRs("/Users/lizhe/Desktop/dataset/DatasetMBRs/MBRs-20", "/Users/lizhe/Desktop/dataset/DatasetMBRs/Bounds-20");
//    PointCloud pc("/Users/lizhe/Desktop/dataset/querySet/Q4-0.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc, 1, "/Users/lizhe/Desktop/reports/final_experiment/5_KCUB.csv", 10000, 10000);
    
    // adjust kcenter amount to 20% and try again
    // change get KNum back to 5% later !!!
    
//    string filepaths[5];
//    filepaths[0] = "/Users/lizhe/Desktop/reports/final_experiment/6_100_GIS2011.csv";
//    filepaths[1] = "/Users/lizhe/Desktop/reports/final_experiment/6_100_KCenter.csv";
//    filepaths[2] = "/Users/lizhe/Desktop/reports/final_experiment/6_100_KCUB.csv";
//    filepaths[3] = "/Users/lizhe/Desktop/reports/final_experiment/6_100_PointBased.csv";
//    filepaths[4] = "/Users/lizhe/Desktop/reports/final_experiment/6_100_PointBasedKcenter.csv";
//    experiment_6_100(filepaths,10);
//////
//    filepaths[0] = "/Users/lizhe/Desktop/reports/final_experiment/6_50_GIS2011.csv";
//    filepaths[1] = "/Users/lizhe/Desktop/reports/final_experiment/6_50_KCenter.csv";
//    filepaths[2] = "/Users/lizhe/Desktop/reports/final_experiment/6_50_KCUB.csv";
//    filepaths[3] = "/Users/lizhe/Desktop/reports/final_experiment/6_50_PointBased.csv";
//    filepaths[4] = "/Users/lizhe/Desktop/reports/final_experiment/6_50_PointBasedKcenter.csv";
//    experiment_6_50(filepaths,10);
//
//    filepaths[0] = "/Users/lizhe/Desktop/reports/final_experiment/6_20_GIS2011.csv";
//    filepaths[1] = "/Users/lizhe/Desktop/reports/final_experiment/6_20_KCenter.csv";
//    filepaths[2] = "/Users/lizhe/Desktop/reports/final_experiment/6_20_KCUB.csv";
//    filepaths[3] = "/Users/lizhe/Desktop/reports/final_experiment/6_20_PointBased.csv";
//    filepaths[4] = "/Users/lizhe/Desktop/reports/final_experiment/6_20_PointBasedKcenter.csv";
//    experiment_6_20(filepaths,10);
//
//    filepaths[0] = "/Users/lizhe/Desktop/reports/final_experiment/6_10_GIS2011.csv";
//    filepaths[1] = "/Users/lizhe/Desktop/reports/final_experiment/6_10_KCenter.csv";
//    filepaths[2] = "/Users/lizhe/Desktop/reports/final_experiment/6_10_KCUB.csv";
//    filepaths[3] = "/Users/lizhe/Desktop/reports/final_experiment/6_10_PointBased.csv";
//    filepaths[4] = "/Users/lizhe/Desktop/reports/final_experiment/6_10_PointBasedKcenter.csv";
//    experiment_6_10(filepaths,10);
    
//    experiment_6_preparement_10();
//    experiment_6_preparement_20();
//    experiment_6_preparement_50();
//    experiment_6_preparement_2();
//    experiment_6_preparement();
    
////     see if need to adjust the PNum
//    string filepaths[5];
//    filepaths[0] = "/Users/lizhe/Desktop/reports/final_experiment/5_GIS2011.csv";
//    filepaths[1] = "/Users/lizhe/Desktop/reports/final_experiment/5_KCenter.csv";
//    filepaths[2] = "/Users/lizhe/Desktop/reports/final_experiment/5_KCUB.csv";
//    filepaths[3] = "/Users/lizhe/Desktop/reports/final_experiment/5_PointBased.csv";
//    filepaths[4] = "/Users/lizhe/Desktop/reports/final_experiment/5_PointBasedKcenter.csv";
//    experiment_5(filepaths, 10);
//
////    string filepaths[5]; // here!!!!!!!1
//    filepaths[0] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=1_GIS2011.csv";
//    filepaths[1] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=1_KCenter.csv";
//    filepaths[2] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=1_KCUB.csv";
//    filepaths[3] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=1_PointBased.csv";
//    filepaths[4] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=1_PointBasedKcenter.csv";
//    experiment_5(filepaths, 1);
//
////    string filepaths[5];
//    filepaths[0] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=2_GIS2011.csv";
//    filepaths[1] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=2_KCenter.csv";
//    filepaths[2] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=2_KCUB.csv";
//    filepaths[3] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=2_PointBased.csv";
//    filepaths[4] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=2_PointBasedKcenter.csv";
//    experiment_5(filepaths, 2);
//
////    string filepaths[5];
//    filepaths[0] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=5_GIS2011.csv";
//    filepaths[1] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=5_KCenter.csv";
//    filepaths[2] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=5_KCUB.csv";
//    filepaths[3] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=5_PointBased.csv";
//    filepaths[4] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=5_PointBasedKcenter.csv";
//    experiment_5(filepaths, 5);
//
////    string filepaths[5];
//    filepaths[0] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=20_GIS2011.csv";
//    filepaths[1] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=20_KCenter.csv";
//    filepaths[2] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=20_KCUB.csv";
//    filepaths[3] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=20_PointBased.csv";
//    filepaths[4] = "/Users/lizhe/Desktop/reports/final_experiment/5_k=20_PointBasedKcenter.csv";
//    experiment_5(filepaths, 20);
    
    
    // first adjust the UB percentage！！！
//    experiment_4("/Users/lizhe/Desktop/reports/final_experiment/4_1.csv", 1);
//    experiment_4("/Users/lizhe/Desktop/reports/final_experiment/4_2.csv", 2);
//    experiment_4("/Users/lizhe/Desktop/reports/final_experiment/4_3.csv", 3);
//    experiment_4("/Users/lizhe/Desktop/reports/final_experiment/4_4.csv", 4);
//    experiment_4("/Users/lizhe/Desktop/reports/final_experiment/4_5.csv", 5);
//    experiment_4("/Users/lizhe/Desktop/reports/final_experiment/4_10.csv", 10);

//    experiment_3_2("/Users/lizhe/Desktop/reports/final_experiment/3_2-N_1-0.01.csv", 0.01);
//    experiment_3_2("/Users/lizhe/Desktop/reports/final_experiment/3_2-N_1-0.02.csv", 0.02);
//    experiment_3_2("/Users/lizhe/Desktop/reports/final_experiment/3_2-N_1-0.05.csv", 0.05);
//    experiment_3_2("/Users/lizhe/Desktop/reports/final_experiment/3_2-N_1-0.10.csv", 0.1);
    
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.001.csv", 0.001);
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.005.csv", 0.005);
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.01.csv", 0.01);
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.02.csv", 0.02);
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.05.csv", 0.05);
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.10.csv", 0.1);
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.20.csv", 0.2);
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.30.csv", 0.3);
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.40.csv", 0.4);
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.50.csv", 0.5);
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.50.csv", 0.6);
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.50.csv", 0.7);
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.50.csv", 0.8);
//    experiment_3("/Users/lizhe/Desktop/reports/final_experiment/3-N_1-0.50.csv", 0.9);
    
//    experiment_2("/Users/lizhe/Desktop/reports/final_experiment/2-0.00.csv", 0.00); // 记得改回来啊！ kcenter 那里至少是1
//    experiment_2("/Users/lizhe/Desktop/reports/final_experiment/2-0.01.csv", 0.01);
//    experiment_2("/Users/lizhe/Desktop/reports/final_experiment/2-0.05.csv", 0.05);
//    experiment_2("/Users/lizhe/Desktop/reports/final_experiment/2-0.10.csv", 0.10);
//    experiment_2("/Users/lizhe/Desktop/reports/final_experiment/2-0.20.csv", 0.20);
//    experiment_2("/Users/lizhe/Desktop/reports/final_experiment/2-0.30.csv", 0.30);
    
//    experiment_1("/Users/lizhe/Desktop/reports/final_experiment/1-2.csv", 2);
//    experiment_1("/Users/lizhe/Desktop/reports/final_experiment/1-5.csv", 5);
//    experiment_1("/Users/lizhe/Desktop/reports/final_experiment/1-10.csv", 10);
//    experiment_1("/Users/lizhe/Desktop/reports/final_experiment/1-20.csv", 20);
//    experiment_1("/Users/lizhe/Desktop/reports/final_experiment/1-50.csv", 50);
//    experiment_1("/Users/lizhe/Desktop/reports/final_experiment/1-100.csv", 100);
    
//
    
//    experiment_1_preparement();
//    prepareQuerySet();
    
    map<string, PointCloud> pointclouds = Dataset::GeneratePointCloudFromTwitterFileWithKeywordWashington("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase");
//    Dataset::StorePointCloudIntoFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington", pointclouds);
    Dataset::StorePointCloudIntoFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-SW", pointclouds);
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington");
    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-SW");
    
    PointCloud pc1 = pointclouds["kid"];
    pc1.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-SW-kid.pts");
    pc1.generateRealWorldPosition("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-SW-kid-real");
    
    PointCloud pc2 = pointclouds["president"];
    pc2.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-SW-president.pts");
    pc2.generateRealWorldPosition("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-SW-president-real");
    
//    PointCloud pc1 = pointclouds["flower"];
//    pc1.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-flower.pts");
//    pc1.generateRealWorldPosition("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-flower-real");
//
//    PointCloud pc2 = pointclouds["love"];
//    pc2.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-love.pts");
//    pc2.generateRealWorldPosition("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-love-real");
//
//    PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-kid.pts");
//    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-young.pts");
//    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-president.pts");
//    PointCloud pc = pointclouds["kid"];
//    pc.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-kid.pts");
//    pc = pointclouds["young"];
//    pc.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-young.pts");
//    pc = pointclouds["balloon"];
//    pc.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-balloon.pts");
//    pc = pointclouds["president"];
//    pc.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-president.pts");
//
//    PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-kid.pts");
//    pc1.calculateCenterPoint();
//    pc1.generateRealWorldPosition("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-kid-real");
//
//    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-young.pts");
//    pc2.calculateCenterPoint();
//    pc2.generateRealWorldPosition("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-young-real");
//
//    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-balloon.pts");
//    pc3.generateRealWorldPosition("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-balloon-real");
//
//    PointCloud pc4("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-president.pts");
//    pc4.calculateCenterPoint();
//    pc4.generateRealWorldPosition("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-president-real");
    
//     PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-Washington-flower.pts");
//    pc5.calculateCenterPoint();
    
//    testGIS2011();
//    map<string, int> keywordIdMap = Dataset::GenerateKeywordIdMapFromOriginal("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase", "/Users/lizhe/Desktop/dataset/keywordIdMap");
//    Dataset::mapKeywordIntoId("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase", "/Users/lizhe/Desktop/dataset/original", keywordIdMap);
//    vector<Point> points = Dataset::RestorePointFromFileWithKeywordId("/Users/lizhe/Desktop/dataset/original");
//    vector<PointCloud> pointclouds = Dataset::GeneratePointCloudFromTwitterFileWithKeywordId("/Users/lizhe/Desktop/dataset/original");
//    Dataset::StorePointCloudIntoFileWithKeywordId("/Users/lizhe/Desktop/dataset/pointclouds", pointclouds);
//    Dataset::Binary_StoreKeywordIdDatasetToFile("/Users/lizhe/Desktop/dataset/pointclouds-binary", pointclouds);
//    Dataset::Binary_StoreKeywordIdPointsToFile("/Users/lizhe/Desktop/dataset/points-binary", points);
    
//    vector<Point> points = Dataset::Binary_RestoreKeywordIdPointsFromFile("/Users/lizhe/Desktop/dataset/points-binary")
    
    
    
//    vector<Point> points;
//    map<int, vector<int>> pidMap;
//    Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary", points, pidMap);
//    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary");
//////    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//
////
////    cout << "dataset size" << dataset.size() << endl;
////
    
//    PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/young.pts");
//    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/president.pts");
//    KNNSearch knn(dataset, points);
//    knn.KNN_Center(pc1, 20);
//    knn.KNN_PAMI2015_Pruning_KCenter(pc1, 20);
//    knn.KNN_BHD(pc1, 20);
//
//    knn.KNN_Center(pc2, 20);
//    knn.KNN_PAMI2015_Pruning_KCenter(pc2, 20);
//    knn.KNN_BHD(pc2, 20);
//    knn.KNN_Center(pc2, 20);
//    knn.KNN_Center(pc4, 20);
//    knn.KNN_Center(pc5, 20);
//    knn.KNN_PAMI2015_Pruning_KCenter(pc, 20);
//    knn.KNN_BHD(pc, 20);
    
////    PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-NewYork-kid.pts");
////    PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-NewYork-young.pts");
//
////
//////    knn.associateMBRs("/Users/lizhe/Desktop/dataset/MBRs", "/Users/lizhe/Desktop/dataset/bounds");
//////    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-10", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound");
////
//    PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/TweetExtract1.pts");
//    knn.KNN_GIS2011(pc1, 10, 10, "/Users/lizhe/Desktop/reports/final_experiment/temp");
//    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/TweetExtract2.pts");
//    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/TweetExtract3.pts");
//    PointCloud pc4("/Users/lizhe/Downloads/ICDE15data/TweetExtract4.pts");
//    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
//    knn.KNN_GIS2011(pc5, 10, 10, "/Users/lizhe/Desktop/reports/final_experiment/temp");
//
////    knn.orderDatasetWithSize(); // 20s
//    knn.buildRtreeForAllPoints(); // 60s
////    knn.generateKeywordIdMap(); // 1s
////    knn.generateKeywordCheck(); // 1s
//
////    knn.KNN_UsingPoint_Efficient_KeywordLB(pc1, 10, pidMap, 2);
////    knn.KNN_PAMI2015_Pruning_KCenter(pc1, 10);
////
////    knn.KNN_UsingPoint_Efficient_KeywordLB(pc2, 10, pidMap, 2);
////    knn.KNN_PAMI2015_Pruning_KCenter(pc2, 10);
////
////    knn.KNN_UsingPoint_Efficient_KeywordLB(pc3, 10, pidMap, 2);
////    knn.KNN_PAMI2015_Pruning_KCenter(pc3, 10);
////
////    knn.KNN_UsingPoint_Efficient_KeywordLB(pc4, 10, pidMap, 2);
////    knn.KNN_PAMI2015_Pruning_KCenter(pc4, 10);
////
////    knn.KNN_UsingPoint_Efficient_KeywordLB(pc5, 10, pidMap, 2);
////    knn.KNN_PAMI2015_Pruning_KCenter(pc5, 10);
//
//    ofstream outfile;
//    outfile.open("/Users/lizhe/Desktop/reports/final_01/using_PointBased.csv", ofstream::app);
//    outfile << endl;
//    outfile << endl;
//    outfile.close();
//
//    for(int i = 0; i < 10; i++){
//        knn.KNN_UsingPoint_Efficient_KeywordLB(pc1, 10, pidMap, 1);
//    }
//
//    outfile.open("/Users/lizhe/Desktop/reports/final_01/using_PointBased.csv", ofstream::app);
//    outfile << endl;
//    outfile << endl;
//    outfile.close();
//
//    for(int i = 0; i < 10; i++){
//        knn.KNN_UsingPoint_Efficient_KeywordLB(pc2, 10, pidMap, 1);
//    }
//
//    outfile.open("/Users/lizhe/Desktop/reports/final_01/using_PointBased.csv", ofstream::app);
//    outfile << endl;
//    outfile << endl;
//    outfile.close();
//
//    for(int i = 0; i < 10; i++){
//        knn.KNN_UsingPoint_Efficient_KeywordLB(pc3, 10, pidMap, 1);
//    }
//
//    outfile.open("/Users/lizhe/Desktop/reports/final_01/using_PointBased.csv", ofstream::app);
//    outfile << endl;
//    outfile << endl;
//    outfile.close();
//
//    for(int i = 0; i < 10; i++){
//        knn.KNN_UsingPoint_Efficient_KeywordLB(pc4, 10, pidMap, 1);
//    }
//
//    outfile.open("/Users/lizhe/Desktop/reports/final_01/using_PointBased.csv", ofstream::app);
//    outfile << endl;
//    outfile << endl;
//    outfile.close();
//
//    for(int i = 0; i < 10; i++){
//        knn.KNN_UsingPoint_Efficient_KeywordLB(pc5, 10, pidMap, 1);
//    }
    
//
//    cout << "test for Q1" << endl;
////
////    knn.KNN_GIS2011(pc1, 10);
////    knn.KNN_PAMI2015_Pruning(pc1, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter(pc1, 10);
//    knn.KNN_BHD(pc1, 20);
////    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10);
////    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc1, 10);
//    knn.KNN_UsingPoint_Efficient(pc1, 10, pidMap, 2);
//
//    cout << "test for Q2" << endl;
//
////    knn.KNN_GIS2011(pc2, 10);
////    knn.KNN_PAMI2015_Pruning(pc2, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter(pc2, 10);
//    knn.KNN_BHD(pc2, 20);
////    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10);
////    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc2, 10);
//    knn.KNN_UsingPoint_Efficient(pc2, 10, pidMap, 2);
//
//    cout << "test for Q3" << endl;
//
////    knn.KNN_GIS2011(pc3, 10);
////    knn.KNN_PAMI2015_Pruning(pc3, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter(pc3, 10);
//    knn.KNN_BHD(pc3, 20);
////    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10);
////    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc3, 10);
//    knn.KNN_UsingPoint_Efficient(pc3, 10, pidMap, 2);
//
//    cout << "test for Q4" << endl;
//
////    knn.KNN_GIS2011(pc4, 10);
////    knn.KNN_PAMI2015_Pruning(pc4, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter(pc4, 10);
////    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10);
////    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc4, 10);
//    knn.KNN_UsingPoint_Efficient(pc4, 10, pidMap, 2);
//
//    cout << "test for Q5" << endl;
//
////    knn.KNN_GIS2011(pc5, 10);
////    knn.KNN_PAMI2015_Pruning(pc5, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter(pc5, 10);
////    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10);
////    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc5, 10);
//    knn.KNN_UsingPoint_Efficient(pc5, 10, pidMap, 2);
    
    
//    vector<PointCloud> data = Dataset::RestorePointCloudFromFileWithKeywordId("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase-OnlyKeywordId-PointClouds");
//    Dataset::Binary_StoreKeywordIdDatasetToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase-OnlyKeywordId-PointClouds-Binary", data);
//    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase-OnlyKeywordId-PointClouds-Binary");
    
//    for(int i = 0; i < data.size(); i++){
//        if(data[i].keywordId == 909120){
//            for(int j = 0; j < data[i].pointcloud.size(); j++){
//                PointCloud pctemp = data[i];
//                if(data[i].pointcloud[j].x == 35.95042563 && data[i].pointcloud[j].y == -83.9331553){
//                    cout << "find it" << endl;
//                    return 0;
//                }
//            }
//        }
//    }
    
    
    
//    vector<Point> points = Dataset::RestorePointFromFileWithKeywordId("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase-KeywordToId");
//    Dataset::Binary_StoreKeywordIdPointsToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase-KeywordToId-Points-Binary", points);
    
//    vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase-OnlyKeywordId-PointClouds-Binary");
//    vector<Point> ps = Dataset::Binary_RestoreKeywordIdPointsFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase-KeywordToId-Points-Binary");

//    PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/TweetExtract1.pts");
//    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/TweetExtract2.pts");
//    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/TweetExtract3.pts");
//    PointCloud pc4("/Users/lizhe/Downloads/ICDE15data/TweetExtract4.pts");
//    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
    
//    PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/food.pts");
//    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/sandwich.pts");
//    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/dog.pts");
//    PointCloud pc4("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/kid.pts");
//    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/flower.pts");
//
//    KNNSearch knn = KNNSearch();
//    knn.dataset = dataset;
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-lowercase-MBRs", "/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-lowercase-Bound");
//    knn.allPoints = ps;
////    knn.orderDatasetWithSize(); // 20s
//    knn.buildRtreeForAllPoints(); // 60s
//    knn.generateKeywordIdMap(); // 1s
//    knn.generateKeywordCheck(); // 1s
//
////    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc1, 10);
////    knn.KNN_PAMI2015_Pruning_KCenter(pc1, 10);
//    knn.NN_UsingPoint_Efficient(pc1);
//
////    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc2, 10);
////    knn.KNN_PAMI2015_Pruning_KCenter(pc2, 10);
//    knn.NN_UsingPoint_Efficient(pc2);
//
////    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc3, 10);
////    knn.KNN_PAMI2015_Pruning_KCenter(pc3, 10);
//    knn.NN_UsingPoint_Efficient(pc3);
//
////    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc4, 10);
////    knn.KNN_PAMI2015_Pruning_KCenter(pc4, 10);
//    knn.NN_UsingPoint_Efficient(pc4);
//
////    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc5, 10);
////    knn.KNN_PAMI2015_Pruning_KCenter(pc5, 10);
//    knn.NN_UsingPoint_Efficient(pc5);
//
    

    
//    Dataset::GenerateKeywordIdMap("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase", "/Users/lizhe/Downloads/ICDE15data/Tweets-keywordIdMap-character-lowercase");
//    map<string, int> keywordIdMap = Dataset::RestoreKeywordIdMap("/Users/lizhe/Downloads/ICDE15data/Tweets-keywordIdMap-character-lowercase");
//    Dataset::mapKeywordIntoId("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase", "/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase-KeywordToId", keywordIdMap);
//    vector<Point> points = Dataset::RestorePointFromFileWithKeywordId("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase-KeywordToId");
    
//    vector<PointCloud> pointclouds = Dataset::GeneratePointCloudFromTwitterFileWithKeywordId("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase-KeywordToId");
//    Dataset::StorePointCloudIntoFileWithKeywordId("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase-OnlyKeywordId-PointClouds", pointclouds);
//    vector<PointCloud> pcs = Dataset::RestorePointCloudFromFileWithKeywordId("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase-OnlyKeywordId-PointClouds");
//    vector<PointCloud> data = Dataset::RestorePointCloudFromFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-USA");
//    PointCloud p1("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/kid-USA.pts");
//    double distance = 0;
//    double max = 0;
//    int index = 0;
//    for(int i = 0; i < data.size(); i++){
//        distance = ExactHausdorff::PAMI2015(p1, data[i]);
//        if(distance > max){
//            max = distance;
//            index = i;
//            cout << "index: " << i << " keyword: " << data[i].keyword << " size: " << data[i].pointcloud.size() << " distance: " << max << endl;
//        }
//    }
    
//    PointCloud p1("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/kid-USA.pts");
//    p1.generateRealWorldPosition("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/kid-USA-T.pts");
//    PointCloud p2("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/young-USA.pts");
//    p2.generateRealWorldPosition("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/yound-USA-T.pts");
//    PointCloud p3("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/balloon-USA.pts");
//    p3.generateRealWorldPosition("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/ballon-USA-T.pts");
    
//    vector<PointCloud> data = Dataset::RestorePointCloudFromFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-USA");
    
//    PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/TweetExtract1.pts");
//    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/TweetExtract2.pts");
//    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/TweetExtract3.pts");
//    PointCloud pc4("/Users/lizhe/Downloads/ICDE15data/TweetExtract4.pts");
//    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
//    KNNSearch knn = KNNSearch();
//    Dataset::GenerateTweetPointCloudsAndAllPoints("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase", knn.dataset, knn.allPoints);
//    knn.orderDatasetWithSize();
//    knn.buildRtreeForAllPoints();
//    knn.generateKeywordIdMap();
//    knn.generateKeywordCheck();
//    knn.KNN_UsingPoint_Efficient(pc1);
//    knn.KNN_UsingPoint_Efficient(pc2);
//    knn.KNN_UsingPoint_Efficient(pc3);
//    knn.KNN_UsingPoint_Efficient(pc4);
//    knn.KNN_UsingPoint_Efficient(pc5);

    
    
    
//    PointCloud pc;
//    for(int i = 0; i < data.size(); i++){
//        if(data[i].keyword == "young"){
//            pc = data[i];
//            pc.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/young-USA.pts");
//        }
//    }
//    cout << "======== search for keyword 'young' =========" << endl;
//    knn.KNN_PAMI2015_Pruning_KCenter(pc, 20);
//    cout << endl;

//    PointCloud pc1, pc2, pc3, pc4, pc5, pc6, pc7, pc8, pc9, pc10, pc11, pc12, pc13, pc14;
//    for(int i = 0; i < data.size(); i++){
//        if(data[i].keyword == "food"){
//            pc1 = data[i];
//            pc1.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/food-USA.pts");
//        }
//        if(data[i].keyword == "sandwich"){
//            pc2 = data[i];
//            pc2.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/sandwich-USA.pts");
//        }
//        if(data[i].keyword == "dog"){
//            pc3 = data[i];
//            pc3.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/dog-USA.pts");
//        }
//        if(data[i].keyword == "kid"){
//            pc4 = data[i];
//            pc4.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/kid-USA.pts");
//        }
//        if(data[i].keyword == "flower"){
//            pc5 = data[i];
//            pc5.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/flower-USA.pts");
//        }
//        if(data[i].keyword == "university"){
//            pc6 = data[i];
//            pc6.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/university-USA.pts");
//        }
//        if(data[i].keyword == "tired"){
//            pc7 = data[i];
//            pc7.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/tired-USA.pts");
//        }
//        if(data[i].keyword == "travel"){
//            pc8 = data[i];
//            pc8.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/travel-USA.pts");
//        }
//        if(data[i].keyword == "hiking"){
//            pc9 = data[i];
//            pc9.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/hiking-USA.pts");
//        }
//        if(data[i].keyword == "iphone"){
//            pc10 = data[i];
//            pc10.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/iphone-USA.pts");
//        }
//        if(data[i].keyword == "buy"){
//            pc11 = data[i];
//            pc11.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/buy-USA.pts");
//        }
//        if(data[i].keyword == "ironman"){
//            pc12 = data[i];
//            pc12.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/ironman-USA.pts");
//        }
//        if(data[i].keyword == "basketball"){
//            pc13 = data[i];
//            pc13.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/basketball-USA.pts");
//        }
//        if(data[i].keyword == "balloon"){
//            pc14 = data[i];
//            pc14.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/balloon-USA.pts");
//        }
//    }
//
//        cout << "======== search for keyword 'food' =========" << endl;
////        PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/food-USA.pts");
//    //    knn.KNN_Center(pc1, 20);
//    //    cout << endl;
//    //    knn.KNN_BHD(pc1, 20);
//    //    cout << endl;
//        knn.KNN_PAMI2015_Pruning_KCenter(pc1, 20);
//        cout << endl;
//    //    knn.KNN_HDLog(pc1, 20);
//    //    cout << endl;
//
//        cout << "======== search for keyword 'sandwich' =========" << endl;
////        PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/sandwich-USA.pts");
//    //    knn.KNN_Center(pc2, 20);
//    //    cout << endl;
//    //    knn.KNN_BHD(pc2, 20);
//    //    cout << endl;
//        knn.KNN_PAMI2015_Pruning_KCenter(pc2, 20);
//        cout << endl;
//    //    knn.KNN_HDLog(pc2, 20);
//    //    cout << endl;
//
//        cout << "======== search for keyword 'dog' =========" << endl;
////        PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/dog-USA.pts");
//    //    knn.KNN_Center(pc3, 20);
//    //    cout << endl;
//    //    knn.KNN_BHD(pc3, 20);
//    //    cout << endl;
//        knn.KNN_PAMI2015_Pruning_KCenter(pc3, 20);
//        cout << endl;
//    //    knn.KNN_HDLog(pc3, 20);
//    //    cout << endl;
//
//        cout << "======== search for keyword 'kid' =========" << endl;
////        PointCloud pc4("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/kid-USA.pts");
//    //    knn.KNN_Center(pc4, 20);
//    //    cout << endl;
//    //    knn.KNN_BHD(pc4, 20);
//    //    cout << endl;
//        knn.KNN_PAMI2015_Pruning_KCenter(pc4, 20);
//        cout << endl;
//    //    knn.KNN_HDLog(pc4, 20);
//    //    cout << endl;
//
//        cout << "======== search for keyword 'flower' =========" << endl;
////        PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/flower-USA.pts");
//    //    knn.KNN_Center(pc5, 20);
//    //    cout << endl;
//    //    knn.KNN_BHD(pc5, 20);
//    //    cout << endl;
//        knn.KNN_PAMI2015_Pruning_KCenter(pc5, 20);
//        cout << endl;
//    //    knn.KNN_HDLog(pc5, 20);
//    //    cout << endl;
//    //
//    cout << "======== search for keyword 'university' =========" << endl;
////    PointCloud pc6("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/university-USA.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc6, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'tired' =========" << endl;
////    PointCloud pc7("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/tired-USA.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc7, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'travel' =========" << endl;
////    PointCloud pc8("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/travel-USA.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc8, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'hiking' =========" << endl;
////    PointCloud pc9("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/hiking-USA.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc9, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'iphone' =========" << endl;
////    PointCloud pc10("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/iphone-USA.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc10, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'buy' =========" << endl;
////    PointCloud pc11("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/buy-USA.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc11, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'ironman' =========" << endl;
////    PointCloud pc12("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/ironman-USA.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc12, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'basketball' =========" << endl;
////    PointCloud pc13("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/basketball-USA.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc13, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'balloon' =========" << endl;
////    PointCloud pc14("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/balloon-USA.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc14, 20);
//    cout << endl;
    
//    map<string, PointCloud> dataset = Dataset::GeneratePointCloudFromTwitterFileWithKeywordUSA("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase");
//    Dataset::StorePointCloudIntoFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase-USA", dataset);
    
//    vector<PointCloud> data = Dataset::RestorePointCloudFromFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase");
//    KNNSearch knn = KNNSearch();
//    knn.dataset = data;
//    knn.generateKeywordMap();
//    knn.buildRtreeForAllPoints();
//    PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/TweetExtract1.pts");
//    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/TweetExtract2.pts");
//    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/TweetExtract3.pts");
//    PointCloud pc4("/Users/lizhe/Downloads/ICDE15data/TweetExtract4.pts");
//    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
//
//    knn.KNN_UsingPoint(pc1);
//    knn.KNN_UsingPoint(pc2);
//    knn.KNN_UsingPoint(pc3);
//    knn.KNN_UsingPoint(pc4);
//    knn.KNN_UsingPoint(pc5);
    
//    PointCloud ref("/Users/lizhe/Downloads/ICDE15data/Second-TweetExtract3.pts");
//    rtree refRTree(ref.pointcloud.begin(), ref.pointcloud.end());
//    std::vector<Point> returned_values;
//    box box_region(Point(30,-90), Point(35,-80));
//    bgi::query(refRTree, bgi::intersects(box_region), std::back_inserter(returned_values));
//    for(int i = 0; i < returned_values.size(); i++){
//        cout << returned_values[i].x << " " << returned_values[i].y << " " << returned_values[i].isAvailable << endl;
//    }
    
    
//    testGIS2011();
    
//    Point p1(2,4);
//    Point p2(1,-1);
//    Point p3(3,3);
//    vector<Point> v;
//    v.push_back(p1);
//    v.push_back(p2);
//    v.push_back(p3);
//    PointCloud pc(v);
//
//    Point pb1(0,0);
//    Point pb2(2,2);
//    pair<Point, Point> pair1(pb1, pb2);
////    double distance = distanceFromPointToMBR(p1, pair1);
////    cout << distance << endl;
//
//    double distance = LowerboundFromKCenterToBound(pc, pair1);
//    cout << distance << endl;
    
//    vector<PointCloud> data = Dataset::RestorePointCloudFromFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase");
//    KNNSearch knn = KNNSearch();
//    knn.dataset = data;
////    knn.generateMBRsForDataset("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-lowercase-MBRs", "/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-lowercase-Bound", data, 10);
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-lowercase-MBRs", "/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-lowercase-Bound");
//
//    ofstream outfile;
//    outfile.open("/Users/lizhe/Desktop/reports/final_01/compare2");
//    int count = 0;
//    double distance1, distance2;
//    for(int i = 0; i < knn.dataset.size(); i++){
//        if(data[i].pointcloud.size() < 1000 && data[i].pointcloud.size() > 100){
//            for(int j = 0; j < knn.dataset.size(); j++){
//                if(data[j].pointcloud.size() < 1000 && data[j].pointcloud.size() > 100){
//                    distance1 = HausdorffDistanceForBound(data[i].bound, data[i].bound);
//                    distance2 = ExactHausdorff::PAMI2015(data[i], data[j]);
//                    if(distance1 != 0 && distance2 > 3*distance1){
//                        outfile << i << " " << data[i].keyword << " " << j << " " << data[j].keyword << " " << distance1 << " " << distance2 << endl;
//                        count++;
//                        if(count == 10){
//                            return 0;
//                        }
//                    }
//                }
//            }
//        }
//    }
    
    
//    map<string, PointCloud> dataset = Dataset::GeneratePointCloudFromTwitterFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase");
//    Dataset::StorePointCloudIntoFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase",dataset);
    
//    Dataset::removeNonCharacter("/Users/lizhe/Downloads/ICDE15data/Tweets", "/Users/lizhe/Downloads/ICDE15data/Tweets-Character");
//    Dataset::transferToLowercase("/Users/lizhe/Downloads/ICDE15data/Tweets-Character", "/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase");
    
//    vector<PointCloud> data = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    vector<PointCloud> data = Dataset::RestorePointCloudFromFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword");
    
//    vector<PointCloud> data = Dataset::RestorePointCloudFromFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-character-lowercase");
//    KNNSearch knn = KNNSearch();
//    knn.dataset = data;
    
//    PointCloud pc1, pc2, pc3, pc4, pc5, pc6, pc7, pc8, pc9, pc10, pc11, pc12, pc13;
//    for(int i = 0; i < data.size(); i++){
//        if(data[i].keyword == "food"){
//            pc1 = data[i];
//            pc1.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/food.pts");
//        }
//        if(data[i].keyword == "sandwich"){
//            pc2 = data[i];
//            pc2.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/sandwich.pts");
//        }
//        if(data[i].keyword == "dog"){
//            pc3 = data[i];
//            pc3.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/dog.pts");
//        }
//        if(data[i].keyword == "kid"){
//            pc4 = data[i];
//            pc4.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/kid.pts");
//        }
//        if(data[i].keyword == "flower"){
//            pc5 = data[i];
//            pc5.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/flower.pts");
//        }
//        if(data[i].keyword == "university"){
//            pc6 = data[i];
//            pc6.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/university.pts");
//        }
//        if(data[i].keyword == "tired"){
//            pc7 = data[i];
//            pc7.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/tired.pts");
//        }
//        if(data[i].keyword == "travel"){
//            pc8 = data[i];
//            pc8.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/travel.pts");
//        }
//        if(data[i].keyword == "hiking"){
//            pc9 = data[i];
//            pc9.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/hiking.pts");
//        }
//        if(data[i].keyword == "iphone"){
//            pc10 = data[i];
//            pc10.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/iphone.pts");
//        }
//        if(data[i].keyword == "buy"){
//            pc11 = data[i];
//            pc11.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/buy.pts");
//        }
//        if(data[i].keyword == "ironman"){
//            pc12 = data[i];
//            pc12.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/ironman.pts");
//        }
//        if(data[i].keyword == "basketball"){
//            pc13 = data[i];
//            pc13.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/basketball.pts");
//        }
//    }
    
//    cout << "======== search for keyword 'food' =========" << endl;
//    PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/food.pts");
//    knn.KNN_Center(pc1, 20);
//    cout << endl;
//    knn.KNN_BHD(pc1, 20);
//    cout << endl;
//    knn.KNN_PAMI2015_Pruning_KCenter(pc1, 20);
//    cout << endl;
//    knn.KNN_HDLog(pc1, 20);
//    cout << endl;

//    cout << "======== search for keyword 'sandwich' =========" << endl;
//    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/sandwich.pts");
//    knn.KNN_Center(pc2, 20);
//    cout << endl;
//    knn.KNN_BHD(pc2, 20);
//    cout << endl;
//    knn.KNN_PAMI2015_Pruning_KCenter(pc2, 20);
//    cout << endl;
//    knn.KNN_HDLog(pc2, 20);
//    cout << endl;

//    cout << "======== search for keyword 'dog' =========" << endl;
//    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/dog.pts");
//    knn.KNN_Center(pc3, 20);
//    cout << endl;
//    knn.KNN_BHD(pc3, 20);
//    cout << endl;
//    knn.KNN_PAMI2015_Pruning_KCenter(pc3, 20);
//    cout << endl;
//    knn.KNN_HDLog(pc3, 20);
//    cout << endl;

//    cout << "======== search for keyword 'kid' =========" << endl;
//    PointCloud pc4("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/kid.pts");
//    knn.KNN_Center(pc4, 20);
//    cout << endl;
//    knn.KNN_BHD(pc4, 20);
//    cout << endl;
//    knn.KNN_PAMI2015_Pruning_KCenter(pc4, 20);
//    cout << endl;
//    knn.KNN_HDLog(pc4, 20);
//    cout << endl;

//    cout << "======== search for keyword 'flower' =========" << endl;
//    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/flower.pts");
//    knn.KNN_Center(pc5, 20);
//    cout << endl;
//    knn.KNN_BHD(pc5, 20);
//    cout << endl;
//    knn.KNN_PAMI2015_Pruning_KCenter(pc5, 20);
//    cout << endl;
//    knn.KNN_HDLog(pc5, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'university' =========" << endl;
//    PointCloud pc6("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/university.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc6, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'tired' =========" << endl;
//    PointCloud pc7("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/tired.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc7, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'travel' =========" << endl;
//    PointCloud pc8("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/travel.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc8, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'hiking' =========" << endl;
//    PointCloud pc9("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/hiking.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc9, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'iphone' =========" << endl;
//    PointCloud pc10("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/iphone.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc10, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'buy' =========" << endl;
//    PointCloud pc11("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/buy.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc11, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'ironman' =========" << endl;
//    PointCloud pc12("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/ironman.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc12, 20);
//    cout << endl;
//
//    cout << "======== search for keyword 'basketball' =========" << endl;
//    PointCloud pc13("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/basketball.pts");
//    knn.KNN_PAMI2015_Pruning_KCenter(pc13, 20);
//    cout << endl;
    
    
//    map<string, PointCloud> dataset = Dataset::GeneratePointCloudFromTwitterFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets");
//    Dataset::StorePointCloudIntoFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword",dataset);
//    vector<PointCloud> data = Dataset::RestorePointCloudFromFileWithKeyword("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword");
//    KNNSearch knn = KNNSearch();
//    knn.dataset = data;
//    knn.generateMBRsForDataset("/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-MBRs", "/Users/lizhe/Downloads/ICDE15data/Tweets-keyword-Bound", data, 10);
//    PointCloud pc1, pc2, pc3, pc4, pc5;
//    for(int i = 0; i < data.size(); i++){
//        if(data[i].keyword == "FOOD"){
//            pc1 = data[i];
//            pc1.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/FOOD.pts");
//        }
//        if(data[i].keyword == "SANDWICH"){
//            pc2 = data[i];
//            pc2.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/SANDWICH.pts");
//        }
//        if(data[i].keyword == "DOG"){
//            pc3 = data[i];
//            pc3.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/DOG.pts");
//        }
//        if(data[i].keyword == "KID"){
//            pc4 = data[i];
//            pc4.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/KID.pts");
//        }
//        if(data[i].keyword == "FLOWER"){
//            pc5 = data[i];
//            pc5.storeToFile("/Users/lizhe/Downloads/ICDE15data/Tweets-RealExample/FLOWER.pts");
//        }
//    }
//    cout << "======== search for keyword FOOD =========" << endl;
//    knn.KNN_PAMI2015_Pruning_KCenter(pc1, 20);
//    cout << "======== search for keyword SANDWICH =========" << endl;
//    knn.KNN_PAMI2015_Pruning_KCenter(pc2, 20);
//    cout << "======== search for keyword DOG =========" << endl;
//    knn.KNN_PAMI2015_Pruning_KCenter(pc3, 20);
//    cout << "======== search for keyword KID =========" << endl;
//    knn.KNN_PAMI2015_Pruning_KCenter(pc4, 20);
//    cout << "======== search for keyword FLOWER =========" << endl;
//    knn.KNN_PAMI2015_Pruning_KCenter(pc5, 20);
    
    
//    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/TweetExtract2.pts");
//    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/TweetExtract3.pts");
//    PointCloud pc4("/Users/lizhe/Downloads/ICDE15data/TweetExtract4.pts");
//    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
//
//    double dis1 = ExactHausdorff::PAMI2015(pc4, pc5);
//    double dis2 = ExactHausdorff::PAMI2015_UsingHilbert(pc4, pc5);
    
//    testGIS2011();
    
//    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
//    pc5.calculateHilbertValue();
//    pc5.myHilbertOrder();
//    int pos = binarySearch(100, 0, pc5.pointcloud.size()-1, pc5);
    
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
    
//    PointCloud ref = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/rtreetest1.pts"); // pass, valid for int
//    for(int i = 0; i < ref.pointcloud.size(); i++){
//        cout << ref.pointcloud[i].x << " " << ref.pointcloud[i].y << endl;
//    }
//    ref.calculateHilbertValue();
//    ref.myHilbertOrder();
//    cout << "after my sorting..." << endl;
//    for(int i = 0; i < ref.pointcloud.size(); i++){
//        cout << ref.pointcloud[i].x << " " << ref.pointcloud[i].y << "\t hilbert value: " << ref.pointcloud[i].hilbertValue << endl;
//    }
//    cout << "after CGAL sorting..." << endl;
//    ref.hilbertOrder();
//    for(int i = 0; i < ref.pointcloud.size(); i++){
//        cout << ref.pointcloud[i].x << " " << ref.pointcloud[i].y << endl;
//    }
    
    
//    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
//    KNNSearch knn = KNNSearch();
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    knn.dataset = dataset;
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/POIs-MBRs-10", "/Users/lizhe/Downloads/ICDE15data/POIs-Bound");
////    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//
////    knn.Test_Time__KNN_PAMI2015_Pruning_KCenter2(pc5, 10);
////    knn.KNN_PAMI2015_Pruning_KCenter(pc5, 10);
////    knn.KthValueRecord_KNN_PAMI2015_Pruning_KCenter(pc5, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter(pc5, 10);
//    knn.KNN_MINE7(pc5, 10, 5, 10);
//    knn.KNN_MINE7_2(pc5, 10, 5, 10);
    
//    testGIS2011();
    
//    PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/TweetExtract1.pts");
//    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/TweetExtract2.pts");
//    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/TweetExtract3.pts");
//    PointCloud pc4("/Users/lizhe/Downloads/ICDE15data/TweetExtract4.pts");
//    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
//
//    PointCloud pc11("/Users/lizhe/Downloads/ICDE15data/TweetExtract1.pts");
//    PointCloud pc22("/Users/lizhe/Downloads/ICDE15data/TweetExtract2.pts");
//    PointCloud pc33("/Users/lizhe/Downloads/ICDE15data/TweetExtract3.pts");
//    PointCloud pc44("/Users/lizhe/Downloads/ICDE15data/TweetExtract4.pts");
//    PointCloud pc55("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
//
//    vector<pair<double,int>> disToKcenter;
//    pc1.sortByKcenter();
//    disToKcenter = pc11.sortByKcenterWithRecord();
//
//    pc2.sortByKcenter();
//    disToKcenter = pc22.sortByKcenterWithRecord();
//
//    pc3.sortByKcenter();
//    disToKcenter = pc33.sortByKcenterWithRecord();
//
//    pc4.sortByKcenter();
//    disToKcenter = pc44.sortByKcenterWithRecord();
//
//    pc5.sortByKcenter();
//    disToKcenter = pc55.sortByKcenterWithRecord();
//
//    cout << "la" << endl;
    
//
//    KNNSearch knn = KNNSearch();
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    knn.dataset = dataset;
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/POIs-MBRs-10", "/Users/lizhe/Downloads/ICDE15data/POIs-Bound");
//    knn.KNN_COMBINED_KCenter_UB(pc5, 10);
//    knn.KNN_COMBINED_KCenter_UB(pc5, 10);
//    knn.KNN_COMBINED_KCenter_UB2(pc5, 10);
//    knn.KNN_COMBINED_KCenter_UB2(pc5, 10);
    

    
    
//    PointCloud ref = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/rtreetest1.pts");
//    ref.generateBoundAndMBRs(10);
    
//    AnalyseLowerBound();
    
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    Dataset::ProcessWithKCenter("/Users/lizhe/Downloads/ICDE15data/POIs-KCenter.pts", dataset);
    
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    Dataset::ProcessWithKCenter("/Users/lizhe/Downloads/ICDE15data/Tweets-KCenter.pts", dataset);
//    testGIS2011();
    
    
//    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
//    pc5.sortByKcenter();
//    PointCloud ref = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/rtreetest1.pts");
//    ref.sortByKcenter();
//    ref.sortByKcenter();
//    ref.sortByKcenter();
//    for(int i = 0; i < ref.pointcloud.size(); i++){
//        cout << i << ":  " << ref.pointcloud[i].x << "  " << ref.pointcloud[i].y << endl;
//    }
//    cout << "lala";
//
//    vector<int> test;
//    test.push_back(0);
//    test.push_back(1);
//    test.push_back(2);
//    test.push_back(3);
//    test.push_back(4);
//    test.push_back(5);
//    test.push_back(6);
//    test.push_back(7);
//    test.push_back(8);
//    test.push_back(9);
//
//    cout << test.size() << endl;
//
//    test[5]=test.back();
//    test.pop_back();
//
//    cout << test.size() << endl;
//    for(int i = 0; i < 10; i++){
//        cout << "test[" << i << "]: " << test[i] << endl;
//    }
//    cout << "=======" << endl;
//
//    for(auto it=test.begin(); it != test.end(); it++){
//        cout << *it << endl;
//    }
    
//    cout << test[4] << endl;
//    cout << test[5] << endl;
//    cout << test[6] << endl;
    
//    Sampling();
//    AnalyseLowerBound();
    
//    computeRandom();
//    testGIS2011_4();
//    testConvergence();
    
//    vector<PointCloud> dataset2 = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    selectRandomPointCloudFromDataset(dataset2);
    
//    testGIS2011();
//    testGIS2011_4();
//    testGIS2011_3();
    
//    KNNSearch knn = KNNSearch();
//    vector<PointCloud> dataset1 = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    knn.generateBoundsForDataset("/Users/lizhe/Downloads/ICDE15data/POIs-Bound", dataset1);
//    vector<PointCloud> dataset2 = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    knn.generateBoundsForDataset("/Users/lizhe/Downloads/ICDE15data/Tweets-Bound", dataset2);
    
//    KNNSearch knn = KNNSearch();
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    knn.dataset = dataset;
//    knn.generateMBRsForDataset("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-Dynamic-10", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound", dataset, 10);
//    knn.generateMBRsForDataset("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-Dynamic-20", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound", dataset, 20);
//    knn.generateMBRsForDataset("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-Dynamic-50", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound", dataset, 50);
//    knn.generateMBRsForDataset("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-Dynamic-100", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound", dataset, 100);
    
//    KNNSearch knn = KNNSearch();
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    knn.dataset = dataset;
//    knn.generateMBRsForDataset("/Users/lizhe/Downloads/ICDE15data/POIs-MBRs-Dynamic-10", "/Users/lizhe/Downloads/ICDE15data/POIs-Bound", dataset, 10);
//    knn.generateMBRsForDataset("/Users/lizhe/Downloads/ICDE15data/POIs-MBRs-Dynamic-20", "/Users/lizhe/Downloads/ICDE15data/POIs-Bound", dataset, 20);
//    knn.generateMBRsForDataset("/Users/lizhe/Downloads/ICDE15data/POIs-MBRs-Dynamic-50", "/Users/lizhe/Downloads/ICDE15data/POIs-Bound", dataset, 50);
//    knn.generateMBRsForDataset("/Users/lizhe/Downloads/ICDE15data/POIs-MBRs-Dynamic-100", "/Users/lizhe/Downloads/ICDE15data/POIs-Bound", dataset, 100);

    
//    PointCloud ref = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/rtreetest1.pts");
//    testMBRs(ref);
    
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    countPointCloudSizeDistribution(dataset);
//    testGIS2011();
//    testGIS2011_2();
    
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    selectRandomPointCloudFromDataset2(dataset);
//    findMaxSize();
//    testGIS2011();
    
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    selectRandomPointCloudFromDataset(dataset);
//    compareGIS2011andPAMI2015(10, 10);
    
//    TEST5();
//    PointCloud ref("/Users/lizhe/Downloads/ICDE15data/sample10000000.pts", 2);
//    PointCloud ref("/Users/lizhe/Downloads/ICDE15data/sample1000.pts", 2);
//    PointCloud ref = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/000020.pts");

//    KNNSearch knn = KNNSearch();
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    knn.dataset = dataset;
//    knn.generateRTreeForDataset("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-100", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound", dataset);
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-10", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound");
//    knn.KNN_GIS2011(ref, 10);
//    knn.KNN_PAMI2015_Pruning(ref, 10);
//    knn.KNN_MINE2(ref, <#int k#>, <#int partialQuerySize#>)
    
    
//    KNNSearch knn = KNNSearch();
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    knn.dataset = dataset;
//    knn.generateRTreeForDataset("/Users/lizhe/Downloads/ICDE15data/POIs-MBRs-100", "/Users/lizhe/Downloads/ICDE15data/POIs-Bound", dataset);
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/POIs-MBRs-10", "/Users/lizhe/Downloads/ICDE15data/POIs-Bound");
//    knn.KNN_GIS2011(ref, 10);
//    knn.KNN_GIS2011(ref, 10);
//    knn.KNN_GIS2011(ref, 10);
//    knn.KNN_PAMI2015_Pruning(ref, 10);
//    knn.KNN_PAMI2015_Pruning(ref, 10);
//    knn.KNN_PAMI2015_Pruning(ref, 10);
//    knn.KNN_MINE(ref, 10, 1, 0.01, 0, 20);
//    knn.KNN_MINE(ref, 10, 2, 0.01, 0, 20);
//    knn.KNN_MINE(ref, 10, 3, 0.01, 0, 20);
//    knn.KNN_GIS_PAMI(ref, 10);
//    knn.KNN_GIS_PAMI(ref, 10);
//    knn.KNN_GIS_PAMI(ref, 10);
    
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    generateRTreeForDataset(dataset, "/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-10");
//    PointCloud ref = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/rtreetest1.pts");
//    testMBRs(ref);
//    TEST4();
//    TEST3();
//    TEST2();
//    TEST();
//    TestQueryandData();
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    generateRandomPointCloudFromDataset(dataset);
    
//    PointCloud ref = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/000020.pts");
//    PointCloud ref = PointCloud("/Users/lizhe/Downloads/ICDE15data/sample_tweet.pts", 2);
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    TestTime(dataset);
//    KNNSearch knn = KNNSearch();
//    knn.dataset = dataset;
//    knn.KNN_PAMI2015_Pruning(ref, 10);
//    knn.Test_Time_KNN_PAMI2015_Pruning(ref,10);
//    knn.Test_Time_KNN_PAMI2015_Pruning2(ref,10);
    
//    KNNTest(4, dataset, 10);
    
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    TestForSingleQueryPointTime(dataset);
    
//    Point p(21,10);
//    vector<Point> ps;
//    ps.push_back(p);
//    PointCloud ref = PointCloud(ps);
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    long count = 0;
//    for(int i = 0; i < dataset.size(); i++){
//        if(dataset[i].pointcloud.size() <= 10)
//            count++;
//    }
//    cout << "1 percentage: " << count/dataset.size() << "  1 numbers:" << count << endl;
    
//    KNNSearch knn = KNNSearch();
//    knn.dataset = dataset;
//    knn.KNN_PAMI2015(ref, 10);
//    knn.KNN_PAMI2015_Pruning(ref, 10);
//    knn.KNN_PR2017_Pruning(ref,10);
//    KNNTest(10, dataset, 10);
    
//    calculateResult("/Users/lizhe/Desktop/reports/Hausdorff6/record2-PAMI2015","/Users/lizhe/Desktop/reports/Hausdorff6/record2-PR2017","/Users/lizhe/Desktop/reports/Hausdorff6/record2-Average");
//    calculateResult("/Users/lizhe/Desktop/reports/Hausdorff6/record-PAMI2015","/Users/lizhe/Desktop/reports/Hausdorff6/record-PR2017","/Users/lizhe/Desktop/reports/Hausdorff6/record-Average");
    
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    KNNTest(10, dataset, 10);
    
//    KNNSearch knn = KNNSearch();
//    knn.dataset = Dataset::GeneratePointCloudFromNationalFile("/Users/lizhe/Downloads/NationalFile_20171201.txt");
    
//    PointCloud ref = PointCloud("/Users/lizhe/Downloads/ICDE15data/ref.pts");
//    PointCloud ref = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/test12.pts");
//    PointCloud ref = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/rtreetest1.pts");
//    PointCloud ref = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/000020.pts");
//    knn.KNN_GIS2011(ref, 10);
//    knn.KNN_PAMI2015(ref, 10);
//    PointCloud ref = PointCloud("/Users/lizhe/Downloads/ICDE15data/ref2.pts");
    
//    knn.dataset = Dataset::GeneratePointCloudFromTwitterFile("/Users/lizhe/Downloads/ICDE15data/Tweets");
//    Dataset::StorePointCloudIntoFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts", knn.dataset);
    
//    knn.dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    knn.KNN_PAMI2015(ref, 10);
//    knn.KNN_PR2017(ref, 10);
    
//    vector<PointCloud> dataset = Dataset::GeneratePointCloudFromPOIFile("/Users/lizhe/Downloads/ICDE15data/POIs");
//    KNNSearch knn = KNNSearch();
//    knn.dataset = dataset;
//    Dataset::StorePointCloudIntoFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts", dataset);
    
//    knn.dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    knn.KNN_PAMI2015(ref, 10);
//    knn.KNN_PR2017(ref, 10);
    
//    vector<PointCloud> dataset = Dataset::GeneratePointCloudFromNationalFile("/Users/lizhe/Downloads/NationalFile_20171201.txt");
//    KNNSearch knn = KNNSearch();
//    knn.dataset = dataset;
//    knn.KNN_PAMI2015(dataset[0], 10);
//    knn.KNN_PR2017(dataset[0], 10);
    
//    PointCloud ref = PointCloud("/Users/lizhe/Downloads/shrec_training/0002-all-1.off");
////    PointCloud ref = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/non-rigid-1625.pts");
//
//    KNNSearch knn = KNNSearch("/Users/lizhe/Downloads/shrec_training"); // non-normalized
////    KNNSearch knn = KNNSearch("/Users/lizhe/Downloads/train_data/02691156"); // normalized
//
////    vector<pair<double,PointCloud>> result = knn.KNN_PAMI2015(ref, 10);
//    vector<pair<double,PointCloud>> result = knn.KNN_PR2017(ref, 10);
//    for(int i = 0; i < 10; i++){
//        cout << result[i].first << endl;
//    }
    
//    Point point = Point(2);
//    cout << point.x << endl;
    
//    PointCloud pc1 = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/Nonnormalized1.pts");
//    PointCloud pc2 = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/Nonnormalized2.pts");
//
//    PointCloud pc3 = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/000020.pts");
//    PointCloud pc4 = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/000907.pts");
//
//    PointCloud pc5 = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/000062.pts");
//    PointCloud pc6 = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/000640.pts");
//
//    PointCloud pc7 = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/non-rigid-1625.pts");
//    PointCloud pc8 = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/non-rigid-1676.pts");
//
//    PointCloud pc9 = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/non-rigid-1509.pts");
//    PointCloud pc10 = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/non-rigid-736.pts");
//

//    compareExactHausdorffDistance(pc9, pc10, true, true, true, false, "");
    
//    double EHD1 = ExactHausdorff::definition(pc1,pc2);
//    double EHD2 = ExactHausdorff::PAMI2015(pc1,pc2);
//    double EHD3 = ExactHausdorff::PR2017(pc1,pc2);
//    cout << "EHD1 : " << EHD1 << endl;
//    cout << "EHD2 : " << EHD2 << endl;
//    cout << "EHD3 : " << EHD3 << endl;
    
    
//    pc1.printAll();
//    pc1.prezorder();
//    for (int i = 0; i < pc1.z_pointcloud.size(); i++){
//        cout << i << " : " << pc1.z_pointcloud[i].x << " " << pc1.z_pointcloud[i].y << " " << pc1.z_pointcloud[i].z << endl;
//    }
//    pc1.zorder();
//    for (int i = 0; i < pc1.pointcloud.size(); i++){
//        cout << i << " : " << pc1.pointcloud[i].zorder << " " << pc1.pointcloud[i].x << " " << pc1.pointcloud[i].y << " " << pc1.pointcloud[i].z << " " << endl;
//    }
    
//    clock_t start, stop, runtime;
//    start = clock();
////    sleep(10);
//    stop = clock();
//    runtime = stop-start;
//    cout << stop - start << " " << runtime << endl;
//    cout << CLOCKS_PER_SEC << endl;
    return 0;
}
