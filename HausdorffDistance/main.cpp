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
using namespace std;

bool cmp_hilbert(pair<Point, int>& hilbertpoint1, pair<Point, int>& hilbertpoint2){
    return hilbertpoint1.second < hilbertpoint2.second;
}

int main(int argc, const char * argv[]) {
    
//    PointCloud pc2("/Users/lizhe/Downloads/ICDE15data/TweetExtract2.pts");
//    PointCloud pc3("/Users/lizhe/Downloads/ICDE15data/TweetExtract3.pts");
//    PointCloud pc4("/Users/lizhe/Downloads/ICDE15data/TweetExtract4.pts");
//    PointCloud pc5("/Users/lizhe/Downloads/ICDE15data/TweetExtract5.pts");
//
//    double dis1 = ExactHausdorff::PAMI2015(pc4, pc5);
//    double dis2 = ExactHausdorff::PAMI2015_UsingHilbert(pc4, pc5);
    
    testGIS2011();
    
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
