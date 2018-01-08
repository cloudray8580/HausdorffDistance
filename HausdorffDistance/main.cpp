//
//  main.cpp
//  HausdorffDistance
//
//  Created by 李喆 on 2018/1/3.
//  Copyright © 2018年 李喆. All rights reserved.
//

#include <iostream>
#include "KNNSearch.hpp"
#include <unistd.h>
using namespace std;

int main(int argc, const char * argv[]) {
    PointCloud pc7 = PointCloud("/Users/lizhe/Desktop/pointclouddataset/other/non-rigid-1625.pts");
    KNNSearch knn = KNNSearch("/Users/lizhe/Downloads/train_data/02691156");
//    vector<pair<double,PointCloud>> result = knn.KNN_PAMI2015(pc7, 10);
    vector<pair<double,PointCloud>> result = knn.KNN_PR2017(pc7, 10);
    for(int i = 0; i < 10; i++){
        cout << result[i].first << endl;
    }
    
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

