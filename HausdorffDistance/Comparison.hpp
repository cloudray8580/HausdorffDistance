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
    KNNSearch knn = KNNSearch();
    srand((unsigned)time(NULL));
    int randomIndex = 0;
    long size;
    vector<PointCloud> querySet;
    for(int i = 0; i < querySetSize;){
        size = dataset.size();
        randomIndex = rand()%size;
        if(dataset[randomIndex].pointcloud.size() < 10){
            continue;
        }
        querySet.push_back(dataset[randomIndex]);
        
        // quick removed
        dataset[randomIndex] = dataset.back();
        dataset.pop_back();
        i++;
    }
    
    for(int i = 0; i < querySetSize; i++){
        cout << "query set size : " << querySet[i].pointcloud.size() << endl;
    }
    
    knn.dataset = dataset;
    clock_t start1, stop1, start2, stop2, start3, stop3, start4, stop4, total1, total2, total3, total4;
    total1 = 0;
    total2 = 0;
    total3 = 0;
    total4 = 0;
    vector<vector<clock_t>> records;
    vector<clock_t> subrecord;
    for (int i = 0; i < querySetSize; i++){
//        start1 = clock();
//        knn.KNN_PAMI2015(querySet[i], k);
//        stop1 = clock();
//        start2 = clock();
//        knn.KNN_PR2017(querySet[i], k);
//        stop2 = clock();
//        start3 = clock();
//        knn.KNN_PAMI2015_Pruning(querySet[i],k);
//        stop3 = clock();
        start4 = clock();
//        knn.KNN_PR2017_Pruning(querySet[i],k);
        knn.Test_Time_KNN_PAMI2015_Pruning(querySet[i],k);
        stop4 = clock();
//        total1 += stop1-start1;
//        total2 += stop2-start2;
//        total3 += stop3-start3;
        total4 += stop4-start4;
        subrecord.clear();
//        subrecord.push_back(stop1-start1);
//        subrecord.push_back(stop2-start2);
//        subrecord.push_back(stop3-start3);
        subrecord.push_back(stop4-start4);
        records.push_back(subrecord);
//        cout << stop1-start1 << " " << stop2-start2 << " " << start3-stop3 << endl;
    }
//    for(int i = 0; i < records.size(); i++){
//        cout << records[i][0] << " " << records[i][1] << " " << records[i][2] << " " << records[i][3] << endl;
//    }
    cout << "average: " << total1/querySetSize << " " << total2/querySetSize << " " << total3/querySetSize << " " << total4/querySetSize << endl;
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
    Point p1(0, 0);
    Point p2(1, 1);
    Point p3(10, 10);
    Point p4(100, 100);
    Point p5(1000, 1000);
    Point p6(1.1, 1.1);
    Point p7(10.1, 10.1);
    Point p8(100.1, 100.1);
    Point p9(1000.1, 1000.1);
    Point p10(99, 88);
    
    vector<Point> ps1, ps2, ps3, ps4, ps5, ps6, ps7, ps8, ps9, ps10;
    ps1.push_back(p1);
    ps2.push_back(p2);
    ps3.push_back(p3);
    ps4.push_back(p4);
    ps5.push_back(p5);
    ps6.push_back(p6);
    ps7.push_back(p7);
    ps8.push_back(p8);
    ps9.push_back(p9);
    ps10.push_back(p10);
    
    PointCloud pc1(ps1), pc2(ps2), pc3(ps3), pc4(ps4), pc5(ps5), pc6(ps6), pc7(ps7), pc8(ps8), pc9(ps9), pc10(ps10);
    vector<PointCloud> qset;
    qset.push_back(pc1);
    qset.push_back(pc2);
    qset.push_back(pc3);
    qset.push_back(pc4);
    qset.push_back(pc5);
    qset.push_back(pc6);
    qset.push_back(pc7);
    qset.push_back(pc8);
    qset.push_back(pc9);
    qset.push_back(pc10);
    
    KNNSearch knn = KNNSearch();
    knn.dataset = dataset;
    
    clock_t start, stop;
    start = clock();
    for(int i = 0; i < 10; i++){
        knn.KNN_PR2017_Pruning(qset[i],10);
    }
    stop = clock();
    cout << "average time for single point point cloud: " << (stop-start)/10 << endl;
}

#endif /* Comparison_hpp */
