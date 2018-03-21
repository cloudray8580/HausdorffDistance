//
//  Util.hpp
//  HausdorffDistance
//
//  Created by 李喆 on 2018/3/8.
//  Copyright © 2018年 李喆. All rights reserved.
//

#ifndef Util_hpp
#define Util_hpp
#include "KNNSearch.hpp"
#include <stdio.h>

void generateRandomPointCloudFromDataset(vector<PointCloud> &dataset){
    int randomIndex1, randomIndex2, randomIndex3;
    long size1 = dataset.size();
    srand((unsigned)time(NULL));
    vector<Point> result1, result2;
    for(int i = 0; i < 10000000; i++){
        randomIndex1 = rand()%size1;
        randomIndex2 = rand()%dataset[randomIndex1].pointcloud.size();
        randomIndex3 = rand()%dataset[randomIndex1].pointcloud.size();
        result1.push_back(dataset[randomIndex1].pointcloud[randomIndex2]);
        result2.push_back(dataset[randomIndex1].pointcloud[randomIndex3]);
    }
    
    PointCloud pc1, pc2;
    pc1.pointcloud = result1;
    pc2.pointcloud = result2;
    pc1.dimension = 2;
    pc2.dimension = 2;
    pc1.storeToFile("/Users/lizhe/Downloads/ICDE15data/sample10000000.pts");
    pc2.storeToFile("/Users/lizhe/Downloads/ICDE15data/data10000000.pts");
}

void selectRandomPointCloudFromDataset(vector<PointCloud> &dataset){
    
    int randomIndex;
    long size1 = dataset.size();
    long size2 = 0;
    PointCloud pc1, pc10, pc100, pc1000, pc10000;
    bool flag1=false, flag10=false, flag100=false, flag1000=false, flag10000=false;
    srand((unsigned)time(NULL));
    while(true){
        randomIndex = rand()%size1;
        size2 = dataset[randomIndex].pointcloud.size();
        if(size2 < 10){
            pc1 = dataset[randomIndex];
            flag1 = true;
        } else if(size2 < 100) {
            pc10 = dataset[randomIndex];
            flag10 = true;
        } else if(size2 < 1000) {
            pc100 = dataset[randomIndex];
            flag100 = true;
        } else if(size2 < 10000) {
            pc1000 = dataset[randomIndex];
            flag1000 = true;
        } else if(size2 < 100000) {
            pc10000 = dataset[randomIndex];
            flag10000 = true;
        }
        
        if(flag1 && flag10 && flag100 && flag1000 && flag10000){
            break;
        }
    }
    pc1.storeToFile("/Users/lizhe/Downloads/ICDE15data/Second-TweetExtract1.pts");
    pc10.storeToFile("/Users/lizhe/Downloads/ICDE15data/Second-TweetExtract2.pts");
    pc100.storeToFile("/Users/lizhe/Downloads/ICDE15data/Second-TweetExtract3.pts");
    pc1000.storeToFile("/Users/lizhe/Downloads/ICDE15data/Second-TweetExtract4.pts");
    pc10000.storeToFile("/Users/lizhe/Downloads/ICDE15data/Second-TweetExtract5.pts");
}

void selectRandomPointCloudFromDataset2(vector<PointCloud> &dataset){
    int randomIndex;
    long size1 = dataset.size();
    long size2 = 0;
    vector<PointCloud> pc2s, pc5s;
    
    srand((unsigned)time(NULL));
    while(true){
        randomIndex = rand()%size1;
        size2 = dataset[randomIndex].pointcloud.size();
        
        if(size2 > 10 && size2 < 50 && pc2s.size() < 5){
            pc2s.push_back(dataset[randomIndex]);
        }
        
        if(size2 > 10000 && size2 < 100000 && pc5s.size() < 5){
            pc5s.push_back(dataset[randomIndex]);
        }
        
        if(pc2s.size() == 5 && pc5s.size() == 5){
            break;
        }
    }
    
    pc2s[0].storeToFile("/Users/lizhe/Downloads/ICDE15data/TweetExtract2-1.pts");
    pc2s[1].storeToFile("/Users/lizhe/Downloads/ICDE15data/TweetExtract2-2.pts");
    pc2s[2].storeToFile("/Users/lizhe/Downloads/ICDE15data/TweetExtract2-3.pts");
    pc2s[3].storeToFile("/Users/lizhe/Downloads/ICDE15data/TweetExtract2-4.pts");
    pc2s[4].storeToFile("/Users/lizhe/Downloads/ICDE15data/TweetExtract2-5.pts");
    
    pc5s[0].storeToFile("/Users/lizhe/Downloads/ICDE15data/TweetExtract5-1.pts");
    pc5s[1].storeToFile("/Users/lizhe/Downloads/ICDE15data/TweetExtract5-2.pts");
    pc5s[2].storeToFile("/Users/lizhe/Downloads/ICDE15data/TweetExtract5-3.pts");
    pc5s[3].storeToFile("/Users/lizhe/Downloads/ICDE15data/TweetExtract5-4.pts");
    pc5s[4].storeToFile("/Users/lizhe/Downloads/ICDE15data/TweetExtract5-5.pts");
    
    for(int i = 0; i < pc2s.size(); i++){
        cout << pc2s[i].pointcloud.size() << endl;
    }
    cout << "miao" << endl;
    for(int i = 0; i < pc5s.size(); i++){
        cout << pc5s[i].pointcloud.size() << endl;
    }
    
}

void countPointCloudSizeDistribution(vector<PointCloud> &dataset){
    
    long size2 = 0;
    int count1 = 0, count2 = 0, count3 = 0, count4 = 0, count5 = 0, count6 = 0, count7 = 0;
    for(int i = 0; i < dataset.size(); i++){
        size2 = dataset[i].pointcloud.size();
        if(size2 < 10){
            count1++;
        } else if(size2 < 100) {
            count2++;
        } else if(size2 < 1000) {
            count3++;
        } else if(size2 < 10000) {
            count4++;
        } else if(size2 < 100000) {
            count5++;
        } else if(size2 < 1000000) {
            count6++;
        } else {
            count7++;
        }
    }
    cout << "count1: " << count1 << "   percentage: " << double(count1)/double(dataset.size()) << endl;
    cout << "count2: " << count2 << "   percentage: " << double(count2)/double(dataset.size()) << endl;
    cout << "count3: " << count3 << "   percentage: " << double(count3)/double(dataset.size()) << endl;
    cout << "count4: " << count4 << "   percentage: " << double(count4)/double(dataset.size()) << endl;
    cout << "count5: " << count5 << "   percentage: " << double(count5)/double(dataset.size()) << endl;
    cout << "count6: " << count6 << "   percentage: " << double(count6)/double(dataset.size()) << endl;
    cout << "count7: " << count7 << "   percentage: " << double(count7)/double(dataset.size()) << endl;
}


#endif /* Util_hpp */
