//
//  Comparison.hpp
//  HausdorffDistance
//
//  Created by 李喆 on 2018/1/3.
//  Copyright © 2018年 李喆. All rights reserved.
//

#ifndef Comparison_hpp
#define Comparison_hpp

#include <stdio.h>
#include <time.h>
#include <fstream>
#include "KNNSearch.hpp"
#include <vector>

using namespace std;

void compareExactHausdorffDistance(PointCloud pc1, PointCloud pc2, bool definition, bool PAMI2015, bool PR2017, bool writetofile, string outputFile){
    
    ofstream outfile;
    if(writetofile){
        outfile.open(outputFile);
        outfile << pc1.pointcloud.size() << " ";
    }
    
    if (definition){
        clock_t start1, stop1, runningtime1;
        start1 = clock();
        double EHD1 = ExactHausdorff::definition(pc1, pc2);
        stop1 = clock();
        runningtime1 = stop1 - start1;
        cout << "definition: " << runningtime1;
        if(writetofile){
            outfile << runningtime1 << " ";
        }
    }
    if(PAMI2015){
        clock_t start2, stop2, runningtime2;
        start2 = clock();
        double EHD2 = ExactHausdorff::PAMI2015(pc1, pc2);
        stop2 = clock();
        runningtime2 = stop2 - start2;
        cout << "   PAMI2015: " << runningtime2;
        if(writetofile){
            outfile << runningtime2 << " ";
        }
    }
    if(PR2017){
        clock_t start3, stop3, runningtime3;
        start3 = clock();
        double EHD3 = ExactHausdorff::PR2017(pc1, pc2);
        stop3 = clock();
        runningtime3 = stop3 - start3;
        cout << "   PR2017: " << runningtime3 << endl;
        if(writetofile){
            outfile << runningtime3 << " ";
        }
    }
    
}

void KNNTest(int querySetSize, vector<PointCloud> &dataset, int k){
//    KNNSearch knn = KNNSearch();
//    srand((unsigned)time(NULL));
//    int randomIndex = 0;
//    long size;
//    vector<PointCloud> querySet;
//    for(int i = 0; i < querySetSize;){
//        size = dataset.size();
//        randomIndex = rand()%size;
//        if(dataset[randomIndex].pointcloud.size() < 10){
//            continue;
//        }
//        querySet.push_back(dataset[randomIndex]);
//
//        // quick removed
//        dataset[randomIndex] = dataset.back();
//        dataset.pop_back();
//        i++;
//    }
//
//    for(int i = 0; i < querySetSize; i++){
//        cout << "query set size : " << querySet[i].pointcloud.size() << endl;
//    }
//
//    knn.dataset = dataset;
//    clock_t start1, stop1, start2, stop2, start3, stop3, start4, stop4, total1, total2, total3, total4;
//    total1 = 0;
//    total2 = 0;
//    total3 = 0;
//    total4 = 0;
//    vector<vector<clock_t>> records;
//    vector<clock_t> subrecord;
//    for (int i = 0; i < querySetSize; i++){
////        start1 = clock();
////        knn.KNN_PAMI2015(querySet[i], k);
////        stop1 = clock();
////        start2 = clock();
////        knn.KNN_PR2017(querySet[i], k);
////        stop2 = clock();
////        start3 = clock();
////        knn.KNN_PAMI2015_Pruning(querySet[i],k);
////        stop3 = clock();
//        start4 = clock();
////        knn.KNN_PR2017_Pruning(querySet[i],k);
//        knn.Test_Time_KNN_PAMI2015_Pruning(querySet[i],k);
//        stop4 = clock();
////        total1 += stop1-start1;
////        total2 += stop2-start2;
////        total3 += stop3-start3;
//        total4 += stop4-start4;
//        subrecord.clear();
////        subrecord.push_back(stop1-start1);
////        subrecord.push_back(stop2-start2);
////        subrecord.push_back(stop3-start3);
//        subrecord.push_back(stop4-start4);
//        records.push_back(subrecord);
////        cout << stop1-start1 << " " << stop2-start2 << " " << start3-stop3 << endl;
//    }
////    for(int i = 0; i < records.size(); i++){
////        cout << records[i][0] << " " << records[i][1] << " " << records[i][2] << " " << records[i][3] << endl;
////    }
//    cout << "average: " << total1/querySetSize << " " << total2/querySetSize << " " << total3/querySetSize << " " << total4/querySetSize << endl;
}

void calculateResult(string resultFilePath1, string resultFilePath2, string outputFilePath){
    ifstream infile;
    infile.open(resultFilePath1);
    string str;
    vector<string> records;
    double preprocess = 0;
    double calculation = 0;
    double sorting = 0;
    double totalPre = 0;
    double totalCal = 0;
    double totalSort = 0;
    int lines = 0;
    while(getline(infile, str)){
        lines++;
        split(records, str, boost::is_any_of(" "));
        preprocess = stod(records[1]);
        calculation = stod(records[2]);
        sorting = stod(records[3]);
        totalPre += preprocess;
        totalCal += calculation;
        totalSort += sorting;
    }
    totalPre /= lines;
    totalCal /= lines;
    totalSort /= lines;
    
    ofstream outfile;
    outfile.open(outputFilePath);
    outfile << totalPre << " " << totalCal << " " << totalSort << endl;
    cout << totalPre << " " << totalCal << " " << totalSort << endl;
    
    infile.close();
    infile.open(resultFilePath2); // if you need to open another file, close the previous first!!!
    preprocess = 0;
    calculation = 0;
    sorting = 0;
    totalPre = 0;
    totalCal = 0;
    totalSort = 0;
    lines = 0;
    while(getline(infile, str)){
        lines++;
        split(records, str, boost::is_any_of(" "));
        preprocess = stod(records[1]);
        calculation = stod(records[2]);
        sorting = stod(records[3]);
        totalPre += preprocess;
        totalCal += calculation;
        totalSort += sorting;
    }
    totalPre /= lines;
    totalCal /= lines;
    totalSort /= lines;
    
    outfile << totalPre << " " << totalCal << " " << totalSort << endl;
    cout << totalPre << " " << totalCal << " " << totalSort << endl;
}

void TestForSingleQueryPointTime(vector<PointCloud> &dataset){
//    Point p1(0, 0);
//    Point p2(1, 1);
//    Point p3(10, 10);
//    Point p4(100, 100);
//    Point p5(1000, 1000);
//    Point p6(1.1, 1.1);
//    Point p7(10.1, 10.1);
//    Point p8(100.1, 100.1);
//    Point p9(1000.1, 1000.1);
//    Point p10(99, 88);
//
//    vector<Point> ps1, ps2, ps3, ps4, ps5, ps6, ps7, ps8, ps9, ps10;
//    ps1.push_back(p1);
//    ps2.push_back(p2);
//    ps3.push_back(p3);
//    ps4.push_back(p4);
//    ps5.push_back(p5);
//    ps6.push_back(p6);
//    ps7.push_back(p7);
//    ps8.push_back(p8);
//    ps9.push_back(p9);
//    ps10.push_back(p10);
//
//    PointCloud pc1(ps1), pc2(ps2), pc3(ps3), pc4(ps4), pc5(ps5), pc6(ps6), pc7(ps7), pc8(ps8), pc9(ps9), pc10(ps10);
//    vector<PointCloud> qset;
//    qset.push_back(pc1);
//    qset.push_back(pc2);
//    qset.push_back(pc3);
//    qset.push_back(pc4);
//    qset.push_back(pc5);
//    qset.push_back(pc6);
//    qset.push_back(pc7);
//    qset.push_back(pc8);
//    qset.push_back(pc9);
//    qset.push_back(pc10);
//
//    KNNSearch knn = KNNSearch();
//    knn.dataset = dataset;
//
//    clock_t start, stop;
//    start = clock();
//    for(int i = 0; i < 10; i++){
//        knn.KNN_PR2017_Pruning(qset[i],10);
//    }
//    stop = clock();
//    cout << "average time for single point point cloud: " << (stop-start)/10 << endl;
}

void TestTime(vector<PointCloud> &dataset){
    
    PointCloud ref1("/Users/lizhe/Downloads/ICDE15data/sample1.pts", 2);
    PointCloud ref10("/Users/lizhe/Downloads/ICDE15data/sample10.pts", 2);
    PointCloud ref100("/Users/lizhe/Downloads/ICDE15data/sample100.pts", 2);
    PointCloud ref1000("/Users/lizhe/Downloads/ICDE15data/sample1000.pts", 2);
    PointCloud ref10000("/Users/lizhe/Downloads/ICDE15data/sample10000.pts", 2);
    
    vector<PointCloud> refs;
    refs.push_back(ref1);
    refs.push_back(ref10);
    refs.push_back(ref100);
    refs.push_back(ref1000);
    refs.push_back(ref10000);
    
    clock_t start, stop;
    for(int i = 0; i < refs.size(); i++){
        start = clock();
        for(int j = 0; j < dataset.size(); j++){
            ExactHausdorff::PAMI2015(refs[i], dataset[j]);
        }
        stop = clock();
        cout << "time usage: " << stop-start << endl;
    }
}

void TestQueryandData(){
    PointCloud ref1("/Users/lizhe/Downloads/ICDE15data/sample1.pts", 2);
    PointCloud ref10("/Users/lizhe/Downloads/ICDE15data/sample10.pts", 2);
    PointCloud ref100("/Users/lizhe/Downloads/ICDE15data/sample100.pts", 2);
    PointCloud ref1000("/Users/lizhe/Downloads/ICDE15data/sample1000.pts", 2);
    PointCloud ref10000("/Users/lizhe/Downloads/ICDE15data/sample10000.pts", 2);
    PointCloud ref100000("/Users/lizhe/Downloads/ICDE15data/sample100000.pts", 2);
    PointCloud ref1000000("/Users/lizhe/Downloads/ICDE15data/sample1000000.pts", 2);
    PointCloud ref10000000("/Users/lizhe/Downloads/ICDE15data/sample10000000.pts", 2);
    
    vector<PointCloud> refs;
    refs.push_back(ref1);
    refs.push_back(ref10);
    refs.push_back(ref100);
    refs.push_back(ref1000);
    refs.push_back(ref10000);
    refs.push_back(ref100000);
    refs.push_back(ref1000000);
    refs.push_back(ref10000000);
    
    PointCloud data1("/Users/lizhe/Downloads/ICDE15data/data1.pts", 2);
    PointCloud data10("/Users/lizhe/Downloads/ICDE15data/data10.pts", 2);
    PointCloud data100("/Users/lizhe/Downloads/ICDE15data/data100.pts", 2);
    PointCloud data1000("/Users/lizhe/Downloads/ICDE15data/data1000.pts", 2);
    PointCloud data10000("/Users/lizhe/Downloads/ICDE15data/data10000.pts", 2);
    PointCloud data100000("/Users/lizhe/Downloads/ICDE15data/data100000.pts", 2);
    PointCloud data1000000("/Users/lizhe/Downloads/ICDE15data/data1000000.pts", 2);
    PointCloud data10000000("/Users/lizhe/Downloads/ICDE15data/data10000000.pts", 2);
    
    vector<PointCloud> datas;
    datas.push_back(data1);
    datas.push_back(data10);
    datas.push_back(data100);
    datas.push_back(data1000);
    datas.push_back(data10000);
    datas.push_back(data100000);
    datas.push_back(data1000000);
    datas.push_back(data10000000);
    
    clock_t start, stop;
    
    vector<vector<clock_t>> records;
    vector<clock_t> subrecord;
    
    for(int count = 0; count < 1; count++){
        for(int i = 0; i < refs.size(); i++){
            subrecord.clear();
            for(int j = 0; j < datas.size(); j++){
                start = clock();
                refs[i].randomize();
                datas[i].randomize();
                ExactHausdorff::PAMI2015(refs[i], datas[j]);
                stop = clock();
    //            cout << stop-start << " ";
                if(count == 0){
                    subrecord.push_back(stop-start);
                } else {
                    records[i][j] += stop-start;
                }
            }
//            cout << endl;
            if(count == 0){
                records.push_back(subrecord);
            }
        }
    }
    
    for(int i = 0; i < records.size(); i++){
        for(int j = 0; j < records[i].size(); j++){
            cout << records[i][j] / 1 << " ";
        }
        cout << endl;
    }
}


double BreakRecord_PAMI2015(PointCloud &pc1, PointCloud &pc2, string filepath){
    
    ofstream outfile;
    outfile.open(filepath);
    
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
    int breakpoint = size2;
    int breakpointCount[11];
    memset(breakpointCount, 0, 10*sizeof(int)); // allocate
    for (int i = 0; i < size1; i++){
        min = std::numeric_limits<double>::infinity();
        breakpoint = size2;
        for(int j = 0; j < size2; j++){
            distance = (*pointcloud1)[i].distanceTo((*pointcloud2)[j]);
            if(distance < min){
                min = distance;
            }
            if(distance <= max){
                breakpoint = j+1;
                break;
            }
        }
        if(min > max){
            max = min;
        }
        breakpointCount[breakpoint]++;
//        outfile << i << " " << breakpoint << endl;
    }
    
    for(int i = 1; i <= 10; i++){
        cout << i << "\t" << breakpointCount[i] << "\t" << (float)breakpointCount[i]*100/size1 << "%" << endl;
        outfile << i << "\t" << breakpointCount[i] << "\t" << (float)breakpointCount[i]*100/size1 << "%" << endl;
    }
    
    return max; // the Hausdorff distance
}

void TEST(){
    PointCloud ref100000("/Users/lizhe/Downloads/ICDE15data/sample100000.pts", 2);
    PointCloud data10("/Users/lizhe/Downloads/ICDE15data/data10.pts", 2);
    clock_t start, stop;
    start = clock();
    ref100000.randomize();
    data10.randomize();
    BreakRecord_PAMI2015(ref100000, data10, "/Users/lizhe/Desktop/reports/KNN5/data1");
    stop = clock();
    cout << "time usage 1: " << stop-start << endl;
}

void TEST2(){
    PointCloud ref1000000("/Users/lizhe/Downloads/ICDE15data/sample1000000.pts", 2);
    PointCloud data10("/Users/lizhe/Downloads/ICDE15data/data10.pts", 2);
    clock_t start, stop;
    start = clock();
    ref1000000.randomize();
    data10.randomize();
    BreakRecord_PAMI2015(ref1000000, data10, "/Users/lizhe/Desktop/reports/KNN5/data2");
    stop = clock();
    cout << "time usage 2: " << stop-start << endl;
}

void TEST3(){
    PointCloud ref100000("/Users/lizhe/Downloads/ICDE15data/sample100000.pts", 2);
    PointCloud ref1000000("/Users/lizhe/Downloads/ICDE15data/sample1000000.pts", 2);
    PointCloud data10("/Users/lizhe/Downloads/ICDE15data/data10.pts", 2);
    clock_t start, stop;
    start = clock();
    ref100000.randomize();
//    data10.randomize();
//    ExactHausdorff::PAMI2015(ref100000, data10);
    stop = clock();
    cout << "time1 " << stop-start << endl;
    
    start = clock();
    ref1000000.randomize();
//    data10.randomize();
//    ExactHausdorff::PAMI2015(ref1000000, data10);
    stop = clock();
    cout << "time2 " << stop-start << endl;
}

void TEST4(){
    PointCloud ref100000("/Users/lizhe/Downloads/ICDE15data/sample100000.pts", 2);
    PointCloud ref1000000("/Users/lizhe/Downloads/ICDE15data/sample1000000.pts", 2);
    PointCloud data10("/Users/lizhe/Downloads/ICDE15data/data10.pts", 2);
    ref100000.randomize();
    ref1000000.randomize();
    data10.randomize();
    BreakRecord_PAMI2015(ref100000, data10, "/Users/lizhe/Desktop/reports/KNN5/data3");
    BreakRecord_PAMI2015(ref1000000, data10, "/Users/lizhe/Desktop/reports/KNN5/data4");
}

void TEST5(){
    PointCloud ref10000000("/Users/lizhe/Downloads/ICDE15data/sample10000000.pts", 2);
    PointCloud data10000000("/Users/lizhe/Downloads/ICDE15data/data10000000.pts", 2);
    clock_t start, start2, stop;
    start = clock();
    ExactHausdorff::PAMI2015(ref10000000, data10000000);
    stop = clock();
    cout << "without randomization: " << stop-start << endl;
    
    start = clock();
    ref10000000.randomize();
    data10000000.randomize();
    start2 = clock();
    ExactHausdorff::PAMI2015(ref10000000, data10000000);
    stop = clock();
    cout << "with randomization: " << stop-start << endl;
    cout << "with randomization but no count randomization time: " << stop-start2 << endl;
}

void compareGIS2011andPAMI2015(int querySetSize, int k){
//    KNNSearch knn = KNNSearch();
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
////    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    
//    srand((unsigned)time(NULL));
//    int randomIndex = 0;
//    long size = dataset.size();
//    vector<PointCloud> querySet;
//    for(int i = 0; i < querySetSize;){
//        size = dataset.size();
//        randomIndex = rand()%size;
//        if(dataset[randomIndex].pointcloud.size() < 100){
//            continue;
//        }
//        querySet.push_back(dataset[randomIndex]);
//        
//        // quick removed
////        dataset[randomIndex] = dataset.back();
////        dataset.pop_back();
//        i++;
//    }
//    
//    knn.dataset = dataset;
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-100", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound");
////    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-10", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound");
////    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/POIs-MBRs-10", "/Users/lizhe/Downloads/ICDE15data/POIs-Bound");
//    
//    for(int i = 0; i < querySetSize; i++){
//        cout << querySet[i].pointcloud.size() << endl;
//    }
//    
//    clock_t start,stop;
//    start = clock();
//    for(int i = 0; i < querySetSize; i++){
//        knn.KNN_GIS2011(querySet[i], k);
//    }
//    stop = clock();
//    cout << "average GIS2011:" << (stop-start)/querySetSize << endl;
//    
//    
//    for(int i = 0; i < querySetSize; i++){
//        querySet[i].randomize();
//    }
//    
//    for(int i = 0; i < dataset.size(); i++){
//        dataset[i].randomize();
//    }
////
//    start = clock();
//    for(int i = 0; i < querySetSize; i++){
//        knn.KNN_PAMI2015_Pruning(querySet[i], k);
//    }
//    stop = clock();
//    cout << "average PAMI2015:" << (stop-start)/querySetSize << endl;
}


void testGIS2011(){
    /*
    PointCloud pc1("/Users/lizhe/Downloads/ICDE15data/TweetExtract1.pts");
    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/TweetExtract2.pts");
    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/TweetExtract3.pts");
    PointCloud pc4("/Users/lizhe/Downloads/ICDE15data/TweetExtract4.pts");
    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
    
//    clock_t start, stop;
//    start = clock();
//    pc5.sortByKcenter();
//    stop = clock();
//    cout << "kcenter 1 time: " << stop-start << endl;
//
//    start = clock();
//    pc5.sortByKcenter();
//    stop = clock();
//    cout << "kcenter 1 time: " << stop-start << endl;
//
//    start = clock();
//    pc5.sortByKcenter();
//    stop = clock();
//    cout << "kcenter 1 time: " << stop-start << endl;
//    
//    start = clock();
//    pc5.sortByKcenter();
//    stop = clock();
//    cout << "kcenter 1 time: " << stop-start << endl;
//
//    start = clock();
//    pc5.sortByKcenter();
//    stop = clock();
//    cout << "kcenter 1 time: " << stop-start << endl;
    
     vector<Point> points;
     map<int, vector<int>> pidMap;
     Dataset::Binary_RestoreKeywordIdPointsFromFile_Seperate("/Users/lizhe/Desktop/dataset/points-binary", points, pidMap);
     vector<PointCloud> dataset = Dataset::Binary_RestoreKeywordIdDatasetFromFile("/Users/lizhe/Desktop/dataset/pointclouds-binary");
     
    KNNSearch knn = KNNSearch();
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets-KCenter.pts");
    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
    knn.dataset = dataset;
    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-10", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound");
//    knn.KNN_GIS2011(pc1, 10);
//    knn.KNN_GIS2011(pc2, 10);
//    knn.KNN_GIS2011(pc3, 10);
//    knn.KNN_GIS2011(pc4, 10);
//    knn.KNN_GIS2011(pc5, 10);

    cout << "=========== segmentation =========" << endl;

    knn.KNN_PAMI2015_Pruning(pc1,10);
    knn.KNN_PAMI2015_Pruning(pc2,10);
    knn.KNN_PAMI2015_Pruning(pc3,10);
    knn.KNN_PAMI2015_Pruning(pc4,10);
    knn.KNN_PAMI2015_Pruning(pc5,10);
//
    cout << "=========== segmentation =========" << endl;

    knn.KNN_PAMI2015_Pruning_KCenter(pc1,10);
    knn.KNN_PAMI2015_Pruning_KCenter(pc2,10);
    knn.KNN_PAMI2015_Pruning_KCenter(pc3,10);
    knn.KNN_PAMI2015_Pruning_KCenter(pc4,10);
    knn.KNN_PAMI2015_Pruning_KCenter(pc5,10);

    cout << "=========== segmentation =========" << endl;
    
    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc1, 10);
    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc2, 10);
    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc3, 10);
    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc4, 10);
    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc5, 10);
    
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc1, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc2, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc3, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc4, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc5, 10, 1000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc1, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc2, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc3, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc4, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBR(pc5, 10, 10000);
    
    cout << "=========== segmentation =========" << endl;
    
    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc1, 10);
    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc2, 10);
    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc3, 10);
    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc4, 10);
    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc5, 10);
    
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc1, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc2, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc3, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc4, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc5, 10, 1000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc1, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc2, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc3, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc4, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_MBRs(pc5, 10, 10000);
    
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertOrder(pc1, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertOrder(pc2, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertOrder(pc3, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertOrder(pc4, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertOrder(pc5, 10);
    
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc1, 10); // 10000
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc2, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc3, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc4, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc5, 10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc1, 10, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc2, 10, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc3, 10, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc4, 10, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc5, 10, 100000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc1, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc2, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc3, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc4, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc5, 10, 1000);
    
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc1, 10, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc2, 10, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc3, 10, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc4, 10, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc5, 10, 100);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc1, 10, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc2, 10, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc3, 10, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc4, 10, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_HilbertUB(pc5, 10, 10);
    
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_orderdata(pc1,10);
//    knn.KNN_PAMI2015_Pruning_KCenter_orderdata(pc2,10);
//    knn.KNN_PAMI2015_Pruning_KCenter_orderdata(pc3,10);
//    knn.KNN_PAMI2015_Pruning_KCenter_orderdata(pc4,10);
//    knn.KNN_PAMI2015_Pruning_KCenter_orderdata(pc5,10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc1, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc2, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc3, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc4, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc5, 10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc1, 10, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc2, 10, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc3, 10, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc4, 10, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc5, 10, 100);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc1, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc2, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc3, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc4, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc5, 10, 1000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc1, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc2, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc3, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc4, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc5, 10, 10000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc1, 10, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc2, 10, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc3, 10, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc4, 10, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB(pc5, 10, 100000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc1, 10, 100000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc2, 10, 100000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc3, 10, 100000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc4, 10, 100000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc5, 10, 100000, 10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc1, 10, 100000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc2, 10, 100000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc3, 10, 100000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc4, 10, 100000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc5, 10, 100000, 100);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc1, 10, 100000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc2, 10, 100000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc3, 10, 100000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc4, 10, 100000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc5, 10, 100000, 1000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc1, 10, 100000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc2, 10, 100000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc3, 10, 100000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc4, 10, 100000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc5, 10, 100000, 10000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc1, 10, 100000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc2, 10, 100000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc3, 10, 100000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc4, 10, 100000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB(pc5, 10, 100000, 100000);
    
//    cout << "===== BscLB = 10 ====" << endl;
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 10, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 10, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 10, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 10, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 10, 10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 100, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 100, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 100, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 100, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 100, 10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 1000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 1000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 1000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 1000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 1000, 10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 10000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 10000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 10000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 10000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 10000, 10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 100000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 100000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 100000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 100000, 10);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 100000, 10);
//
//    cout << "===== BscLB = 100 ====" << endl;
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 10, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 10, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 10, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 10, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 10, 100);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 100, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 100, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 100, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 100, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 100, 100);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 1000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 1000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 1000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 1000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 1000, 100);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 10000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 10000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 10000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 10000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 10000, 100);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 100000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 100000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 100000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 100000, 100);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 100000, 100);
//
//    cout << "===== BscLB = 1000 ====" << endl;
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 10, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 10, 1000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 100, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 100, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 100, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 100, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 100, 1000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 1000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 1000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 1000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 1000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 1000, 1000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 10000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 10000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 10000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 10000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 10000, 1000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 100000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 100000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 100000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 100000, 1000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 100000, 1000);
//
//    cout << "===== BscLB = 10000 ====" << endl;
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 10, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 10, 10000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 100, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 100, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 100, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 100, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 100, 10000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 1000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 1000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 1000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 1000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 1000, 10000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 10000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 10000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 10000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 10000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 10000, 10000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 100000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 100000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 100000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 100000, 10000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 100000, 10000);
//
//    cout << "===== BscLB = 100,000 ====" << endl;
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 10, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 10, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 10, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 10, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 10, 100000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 100, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 100, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 100, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 100, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 100, 100000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 1000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 1000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 1000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 1000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 1000, 100000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 10000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 10000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 10000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 10000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 10000, 100000);
//
    cout << "=========== segmentation =========" << endl;
    
    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc1, 10, 100000, 100000);
    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc2, 10, 100000, 100000);
    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc3, 10, 100000, 100000);
    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc4, 10, 100000, 100000);
    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(pc5, 10, 100000, 100000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB3(pc1, 10, 100000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB3(pc2, 10, 100000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB3(pc3, 10, 100000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB3(pc4, 10, 100000, 100000);
//    knn.KNN_PAMI2015_Pruning_KCenter_UB_BscLB3(pc5, 10, 100000, 100000);
    
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_COMBINED(pc1,10);
//    knn.KNN_COMBINED(pc2,10);
//    knn.KNN_COMBINED(pc3,10);
//    knn.KNN_COMBINED(pc4,10);
//    knn.KNN_COMBINED(pc5,10);
////
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_COMBINED_KCenter(pc1,10);
//    knn.KNN_COMBINED_KCenter(pc2,10);
//    knn.KNN_COMBINED_KCenter(pc3,10);
//    knn.KNN_COMBINED_KCenter(pc4,10);
//    knn.KNN_COMBINED_KCenter(pc5,10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_COMBINED_KCenter_UB(pc1, 10);
//    knn.KNN_COMBINED_KCenter_UB(pc2, 10);
//    knn.KNN_COMBINED_KCenter_UB(pc3, 10);
//    knn.KNN_COMBINED_KCenter_UB(pc4, 10);
//    knn.KNN_COMBINED_KCenter_UB(pc5, 10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_COMBINED_KCenter_UB2(pc1, 10);
//    knn.KNN_COMBINED_KCenter_UB2(pc2, 10);
//    knn.KNN_COMBINED_KCenter_UB2(pc3, 10);
//    knn.KNN_COMBINED_KCenter_UB2(pc4, 10);
//    knn.KNN_COMBINED_KCenter_UB2(pc5, 10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_COMBINED_KCenter_UB2(pc1, 10, 10, 100);
//    knn.KNN_COMBINED_KCenter_UB2(pc2, 10, 10, 100);
//    knn.KNN_COMBINED_KCenter_UB2(pc3, 10, 10, 100);
//    knn.KNN_COMBINED_KCenter_UB2(pc4, 10, 10, 100);
//    knn.KNN_COMBINED_KCenter_UB2(pc5, 10, 10, 100);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_COMBINED_KCenter_UB2(pc1, 10, 10, 1000);
//    knn.KNN_COMBINED_KCenter_UB2(pc2, 10, 10, 1000);
//    knn.KNN_COMBINED_KCenter_UB2(pc3, 10, 10, 1000);
//    knn.KNN_COMBINED_KCenter_UB2(pc4, 10, 10, 1000);
//    knn.KNN_COMBINED_KCenter_UB2(pc5, 10, 10, 1000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_COMBINED_KCenter_UB2(pc1, 10, 10, 10000);
//    knn.KNN_COMBINED_KCenter_UB2(pc2, 10, 10, 10000);
//    knn.KNN_COMBINED_KCenter_UB2(pc3, 10, 10, 10000);
//    knn.KNN_COMBINED_KCenter_UB2(pc4, 10, 10, 10000);
//    knn.KNN_COMBINED_KCenter_UB2(pc5, 10, 10, 10000);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_COMBINED_KCenter_UB2(pc1, 10, 10, 100000);
//    knn.KNN_COMBINED_KCenter_UB2(pc2, 10, 10, 100000);
//    knn.KNN_COMBINED_KCenter_UB2(pc3, 10, 10, 100000);
//    knn.KNN_COMBINED_KCenter_UB2(pc4, 10, 10, 100000);
//    knn.KNN_COMBINED_KCenter_UB2(pc5, 10, 10, 100000);
    
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_COMBINED_KCenter_LB(pc1, 10);
//    knn.KNN_COMBINED_KCenter_LB(pc2, 10);
//    knn.KNN_COMBINED_KCenter_LB(pc3, 10);
//    knn.KNN_COMBINED_KCenter_LB(pc4, 10);
//    knn.KNN_COMBINED_KCenter_LB(pc5, 10);
    
//    cout << "=========== segmentation =========" << endl;
//    // the result is not right !
//    knn.KNN_MINE(pc1, 10, 1, 0.01, 0, 20);
//    knn.KNN_MINE(pc2, 10, 1, 0.01, 0, 20);
//    knn.KNN_MINE(pc3, 10, 1, 0.01, 0, 20);
//    knn.KNN_MINE(pc4, 10, 1, 0.01, 0, 20);
//    knn.KNN_MINE(pc5, 10, 1, 0.01, 0, 20);
    
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_MINE2(pc1, 10, 20);
//    knn.KNN_MINE2(pc2, 10, 20);
//    knn.KNN_MINE2(pc3, 10, 20);
//    knn.KNN_MINE2(pc4, 10, 20);
//    knn.KNN_MINE2(pc5, 10, 20);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_MINE3(pc1, 10, 0.05, 20, 1000, 10);
//    knn.KNN_MINE3(pc2, 10, 0.05, 20, 1000, 10);
//    knn.KNN_MINE3(pc3, 10, 0.05, 20, 1000, 10);
//    knn.KNN_MINE3(pc4, 10, 0.05, 20, 1000, 10);
//    knn.KNN_MINE3(pc5, 10, 0.05, 20, 1000, 10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_MINE4(pc1, 10, 0.05, 20, 100, 10);
//    knn.KNN_MINE4(pc2, 10, 0.05, 20, 100, 10);
//    knn.KNN_MINE4(pc3, 10, 0.05, 20, 100, 10);
//    knn.KNN_MINE4(pc4, 10, 0.05, 20, 100, 10);
//    knn.KNN_MINE4(pc5, 10, 0.05, 20, 100, 10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_MINE5(pc1, 10, 0.05, 20, 100, 10, 0.000316229);
//    knn.KNN_MINE5(pc2, 10, 0.05, 20, 100, 10, 0.032309);
//    knn.KNN_MINE5(pc3, 10, 0.05, 20, 100, 10, 0.273380);
//    knn.KNN_MINE5(pc4, 10, 0.05, 20, 100, 10, 0.187096);
//    knn.KNN_MINE5(pc5, 10, 0.05, 20, 100, 10, 0.929885); // increase the threshold a little to avoid missing the true KNN
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_MINE6(pc1, 10, 5, 10);
//    knn.KNN_MINE6(pc2, 10, 5, 10);
//    knn.KNN_MINE6(pc3, 10, 5, 10);
//    knn.KNN_MINE6(pc4, 10, 5, 10);
//    knn.KNN_MINE6(pc5, 10, 5, 10);
    
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_MINE6(pc1, 10, 5, 10, 0.5, 20, 1000);
//    knn.KNN_MINE6(pc2, 10, 5, 10, 0.5, 20, 1000);
//    knn.KNN_MINE6(pc3, 10, 5, 10, 0.5, 20, 1000);
//    knn.KNN_MINE6(pc4, 10, 5, 10, 0.5, 20, 1000);
//    knn.KNN_MINE6(pc5, 10, 5, 10, 0.5, 20, 1000);
    
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_MINE7(pc1, 10, 5, 10);
//    knn.KNN_MINE7(pc2, 10, 5, 10);
//    knn.KNN_MINE7(pc3, 10, 5, 10);
//    knn.KNN_MINE7(pc4, 10, 5, 10);
//    knn.KNN_MINE7(pc5, 10, 5, 10);
    
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_MINE7_2(pc1, 10, 5, 10);
//    knn.KNN_MINE7_2(pc2, 10, 5, 10);
//    knn.KNN_MINE7_2(pc3, 10, 5, 10);
//    knn.KNN_MINE7_2(pc4, 10, 5, 10);
//    knn.KNN_MINE7_2(pc5, 10, 5, 10);
//
//    cout << "=========== segmentation =========" << endl;
//
//    knn.KNN_MINE7_2(pc1, 10, 5, 10, 0.05, 20, 100, 100000);
//    knn.KNN_MINE7_2(pc2, 10, 5, 10, 0.05, 20, 100, 100000);
//    knn.KNN_MINE7_2(pc3, 10, 5, 10, 0.05, 20, 100, 100000);
//    knn.KNN_MINE7_2(pc4, 10, 5, 10, 0.05, 20, 100, 100000);
//    knn.KNN_MINE7_2(pc5, 10, 5, 10, 0.05, 20, 100, 100000);
    
//    cout << "=========== segmentation =========" << endl;
//    
//    knn.KNN_MINE7_2(pc1, 10, 20, 10, 0.05, 20, 100, 100000);
//    knn.KNN_MINE7_2(pc2, 10, 20, 10, 0.05, 20, 100, 100000);
//    knn.KNN_MINE7_2(pc3, 10, 20, 10, 0.05, 20, 100, 100000);
//    knn.KNN_MINE7_2(pc4, 10, 20, 10, 0.05, 20, 100, 100000);
//    knn.KNN_MINE7_2(pc5, 10, 20, 10, 0.05, 20, 100, 100000);
    
//    cout << "=========== segmentation =========" << endl;
//    
//    knn.KNN_MINE7_3(pc1, 10, 20, 10, 0.05, 20, 100, 100000);
//    knn.KNN_MINE7_3(pc2, 10, 20, 10, 0.05, 20, 100, 100000);
//    knn.KNN_MINE7_3(pc3, 10, 20, 10, 0.05, 20, 100, 100000);
//    knn.KNN_MINE7_3(pc4, 10, 20, 10, 0.05, 20, 100, 100000);
//    knn.KNN_MINE7_3(pc5, 10, 20, 10, 0.05, 20, 100, 100000);
     
     */
}


void testGIS2011_2(){
    
//    PointCloud pc21("/Users/lizhe/Downloads/ICDE15data/TweetExtract2-1.pts");
//    PointCloud pc22("/Users/lizhe/Downloads/ICDE15data/TweetExtract2-2.pts");
//    PointCloud pc23("/Users/lizhe/Downloads/ICDE15data/TweetExtract2-3.pts");
//    PointCloud pc24("/Users/lizhe/Downloads/ICDE15data/TweetExtract2-4.pts");
//    PointCloud pc25("/Users/lizhe/Downloads/ICDE15data/TweetExtract2-5.pts");
//
//    PointCloud pc51("/Users/lizhe/Downloads/ICDE15data/TweetExtract5-1.pts");
//    PointCloud pc52("/Users/lizhe/Downloads/ICDE15data/TweetExtract5-2.pts");
//    PointCloud pc53("/Users/lizhe/Downloads/ICDE15data/TweetExtract5-3.pts");
//    PointCloud pc54("/Users/lizhe/Downloads/ICDE15data/TweetExtract5-4.pts");
//    PointCloud pc55("/Users/lizhe/Downloads/ICDE15data/TweetExtract5-5.pts");
//
//    KNNSearch knn = KNNSearch();
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    knn.dataset = dataset;
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-10", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound");
//
//    knn.KNN_GIS2011(pc21, 10);
//    knn.KNN_GIS2011(pc22, 10);
//    knn.KNN_GIS2011(pc23, 10);
//    knn.KNN_GIS2011(pc24, 10);
//    knn.KNN_GIS2011(pc25, 10);
//
//    knn.KNN_GIS2011(pc51, 10);
//    knn.KNN_GIS2011(pc52, 10);
//    knn.KNN_GIS2011(pc53, 10);
//    knn.KNN_GIS2011(pc54, 10);
//    knn.KNN_GIS2011(pc55, 10);
//}
//
//void testGIS2011_3(){
//    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/TweetExtract3.pts");
//    KNNSearch knn = KNNSearch();
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    knn.dataset = dataset;
//
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-Dynamic-10", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound");
//    knn.KNN_GIS2011(pc3, 10);
//
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-Dynamic-20", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound");
//    knn.KNN_GIS2011(pc3, 10);
//
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-Dynamic-50", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound");
//    knn.KNN_GIS2011(pc3, 10);
//
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-Dynamic-100", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound");
//    knn.KNN_GIS2011(pc3, 10);
}

void testGIS2011_4(){
//    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/TweetExtract3.pts");
//    KNNSearch knn = KNNSearch();
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    knn.dataset = dataset;
//
////    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/POIs-MBRs-10", "/Users/lizhe/Downloads/ICDE15data/POIs-Bound");
////    knn.KNN_GIS2011(pc3, 10);
//
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/POIs-MBRs-Dynamic-10", "/Users/lizhe/Downloads/ICDE15data/POIs-Bound");
//    knn.KNN_GIS2011(pc3, 10);
//    knn.KNN_COMBINED(pc3,10);
//    knn.KNN_PAMI2015_Pruning(pc3,10);
//    knn.KNN_MINE3(pc3, 10, 0.05, 20, 1000, 10);
////    knn.KNN_MINE2(pc3, 10, 20);
//    knn.KNN_PAMI2015(pc3, 10);
}

void testConvergence(){
    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/TweetExtract2.pts");
    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/TweetExtract3.pts");
    PointCloud pc4("/Users/lizhe/Downloads/ICDE15data/TweetExtract4.pts");
    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
    
    PointCloud data1("/Users/lizhe/Downloads/ICDE15data/Second-TweetExtract1.pts");
    PointCloud data2("/Users/lizhe/Downloads/ICDE15data/Second-TweetExtract2.pts");
    PointCloud data3("/Users/lizhe/Downloads/ICDE15data/Second-TweetExtract3.pts");
    PointCloud data4("/Users/lizhe/Downloads/ICDE15data/Second-TweetExtract4.pts");
    PointCloud data5("/Users/lizhe/Downloads/ICDE15data/Second-TweetExtract5.pts");
    
    ExactHausdorff::PAMI2015_recordMax(pc2, data1, "/Users/lizhe/Desktop/reports/KNN7/convergence2-1");
    ExactHausdorff::PAMI2015_recordMax(pc2, data2, "/Users/lizhe/Desktop/reports/KNN7/convergence2-2");
    ExactHausdorff::PAMI2015_recordMax(pc2, data3, "/Users/lizhe/Desktop/reports/KNN7/convergence2-3");
    ExactHausdorff::PAMI2015_recordMax(pc2, data4, "/Users/lizhe/Desktop/reports/KNN7/convergence2-4");
    ExactHausdorff::PAMI2015_recordMax(pc2, data5, "/Users/lizhe/Desktop/reports/KNN7/convergence2-5");
    
    ExactHausdorff::PAMI2015_recordMax(pc3, data1, "/Users/lizhe/Desktop/reports/KNN7/convergence3-1");
    ExactHausdorff::PAMI2015_recordMax(pc3, data2, "/Users/lizhe/Desktop/reports/KNN7/convergence3-2");
    ExactHausdorff::PAMI2015_recordMax(pc3, data3, "/Users/lizhe/Desktop/reports/KNN7/convergence3-3");
    ExactHausdorff::PAMI2015_recordMax(pc3, data4, "/Users/lizhe/Desktop/reports/KNN7/convergence3-4");
    ExactHausdorff::PAMI2015_recordMax(pc3, data5, "/Users/lizhe/Desktop/reports/KNN7/convergence3-5");
    
    ExactHausdorff::PAMI2015_recordMax(pc4, data1, "/Users/lizhe/Desktop/reports/KNN7/convergence4-1");
    ExactHausdorff::PAMI2015_recordMax(pc4, data2, "/Users/lizhe/Desktop/reports/KNN7/convergence4-2");
    ExactHausdorff::PAMI2015_recordMax(pc4, data3, "/Users/lizhe/Desktop/reports/KNN7/convergence4-3");
    ExactHausdorff::PAMI2015_recordMax(pc4, data4, "/Users/lizhe/Desktop/reports/KNN7/convergence4-4");
    ExactHausdorff::PAMI2015_recordMax(pc4, data5, "/Users/lizhe/Desktop/reports/KNN7/convergence4-5");
    
    ExactHausdorff::PAMI2015_recordMax(pc5, data1, "/Users/lizhe/Desktop/reports/KNN7/convergence5-1");
    ExactHausdorff::PAMI2015_recordMax(pc5, data2, "/Users/lizhe/Desktop/reports/KNN7/convergence5-2");
    ExactHausdorff::PAMI2015_recordMax(pc5, data3, "/Users/lizhe/Desktop/reports/KNN7/convergence5-3");
    ExactHausdorff::PAMI2015_recordMax(pc5, data4, "/Users/lizhe/Desktop/reports/KNN7/convergence5-4");
    ExactHausdorff::PAMI2015_recordMax(pc5, data5, "/Users/lizhe/Desktop/reports/KNN7/convergence5-5");
    
}


void findMaxSize(){
    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
    int max = 0;
    for(int i = 0; i < dataset.size(); i++){
        if(dataset[i].pointcloud.size() > max){
            max = dataset[i].pointcloud.size();
        }
    }
    cout << "max size = " << max << endl;
}

void computeRandom(){
    PointCloud pc1(10000, 0, 10, 0, 10);
    sleep(10);
    PointCloud pc2(10000, 0, 10, 0, 10);
    
    pc1.randomize();
    pc2.randomize();
    
    PointCloud pc3(pc1, 1000);
    PointCloud pc4(pc2, 1000);
    
    clock_t start, stop;
    start = clock();
    double distance = ExactHausdorff::PAMI2015(pc3,pc4);
    stop = clock();
    cout << distance << " " << stop-start << endl;
}

void AnalyseLowerBound(){
//    KNNSearch knn = KNNSearch();
//    
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    knn.dataset = dataset;
//    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/Tweets-MBRs-10", "/Users/lizhe/Downloads/ICDE15data/Tweets-Bound");
//    
////    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
////    knn.dataset = dataset;
////    knn.associateMBRs("/Users/lizhe/Downloads/ICDE15data/POIs-MBRs-10", "/Users/lizhe/Downloads/ICDE15data/POIs-Bound");
//    
//    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
//    knn.AnalyseLBs(pc5);
}

void Sampling(){
    ifstream infile1, infile2, infile3, infile4;
    ofstream outfile1, outfile2, outfile3, outfile4;
    
    infile1.open("/Users/lizhe/Desktop/reports/KNN8/sort-bsclb-node");
    outfile1.open("/Users/lizhe/Desktop/reports/KNN8/sort-bsclb-node-sample");
    string line;
    int count = 0;
    while(getline(infile1,line)){
        count++;
        if(count % 100 == 0){
            outfile1 << line << endl;
        }
    }
    infile1.close();
    outfile1.close();
    
    infile2.open("/Users/lizhe/Desktop/reports/KNN8/sort-enhlb-node");
    outfile2.open("/Users/lizhe/Desktop/reports/KNN8/sort-enhlb-node-sample");
    count = 0;
    while(getline(infile2,line)){
        count++;
        if(count % 100 == 0){
            outfile2 << line << endl;
        }
    }
    infile2.close();
    outfile2.close();
    
    infile3.open("/Users/lizhe/Desktop/reports/KNN8/sort-partialHD-node");
    outfile3.open("/Users/lizhe/Desktop/reports/KNN8/sort-partialHD-node-sample");
    count = 0;
    while(getline(infile3,line)){
        count++;
        if(count % 100 == 0){
            outfile3 << line << endl;
        }
    }
    infile3.close();
    outfile3.close();
    
    infile4.open("/Users/lizhe/Desktop/reports/KNN8/sort-exactHD-node");
    outfile4.open("/Users/lizhe/Desktop/reports/KNN8/sort-exactHD-node-sample");
    count = 0;
    while(getline(infile4,line)){
        count++;
        if(count % 100 == 0){
            outfile4 << line << endl;
        }
    }
    infile4.close();
    outfile4.close();
}

#endif /* Comparison_hpp */
