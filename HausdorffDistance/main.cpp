//
//  main.cpp
//  HausdorffDistance
//
//  Created by 李喆 on 2018/1/3.
//  Copyright © 2018年 李喆. All rights reserved.
//

#include <iostream>
#include "Comparison.hpp"
using namespace std;

int main(int argc, const char * argv[]) {
    
//    PointCloud ref = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/000020.pts");
//    PointCloud ref = PointCloud("/Users/lizhe/Downloads/ICDE15data/sample_tweet.pts", 2);
    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/Tweets.pts");
//    vector<PointCloud> dataset = Dataset::RestorePointCloudFromFile("/Users/lizhe/Downloads/ICDE15data/POIs.pts");
//    KNNSearch knn = KNNSearch();
//    knn.dataset = dataset;
//    knn.KNN_PAMI2015_Pruning(ref, 10);
//    knn.Test_Time_KNN_PAMI2015_Pruning(ref,10);
//    knn.Test_Time_KNN_PAMI2015_Pruning2(ref,10);
    
    KNNTest(4, dataset, 10);
    
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
