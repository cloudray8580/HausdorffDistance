//
//  KNNSearch.hpp
//  HausdorffDistance
//
//  Created by 李喆 on 2018/1/3.
//  Copyright © 2018年 李喆. All rights reserved.
//

#ifndef KNNSearch_hpp
#define KNNSearch_hpp
#include "ExactHausdorff.hpp"
#include <stdio.h>

#include <sys/types.h>
#include <dirent.h>

class KNNSearch{
public:
    
    KNNSearch(vector<PointCloud> &dataset, vector<Point> &points):dataset(dataset), allPoints(points){}
    
//    KNNSearch(){
//        vector<PointCloud> pcs;
//        this->dataset = pcs;
//        vector<Point> ps;
//        this->allPoints = ps;
//    }
//
//    KNNSearch(vector<PointCloud> &dataset, vector<Point> &points){
//        this->dataset = dataset;
//        this->allPoints = points;
//    }
    
//    KNNSearch(string directory);
    
    vector<PointCloud> &dataset;
    vector<Point> &allPoints;
    map<string, int> keywordMapForDataset;
    map<int, int> keywordIdMapForDataset; // first keywordId, second point cloud index
    rtree refRTree;
    bool* keywordCheck; // should have size of dataset
    
    void randomizeDataset();
    void loadDataset(string directory);
    void generateBoundsForDataset(string filepath, vector<PointCloud> &dataset);
    void generateRTreeForDataset(string filepath1, string filepath2, vector<PointCloud> &dataset);
    void generateMBRsForDataset(string filepath1, string filepath2, vector<PointCloud> &dataset, int number);
    void associateMBRs(string filepath1, string filepath2);
    void generateKeywordId(map<string,int> keywordmap);
    void generateKeywordMap(); // need to bind dataset first;
    void generateKeywordIdMap();
    void generateKeywordCheck();
    void buildRtreeForAllPoints();
    void orderDatasetWithSize();
    
    vector<pair<double,PointCloud>> KNN_PAMI2015(PointCloud &ref, int k);
    vector<pair<double,PointCloud>> KNN_PR2017(PointCloud &ref, int k);
    vector<tempResult> KNN_GIS2011(PointCloud &ref, int k, int number = 10);
    
    void KNN_PAMI2015_Pruning(PointCloud &ref, int k); // use a priority queue to keep K
    void KNN_PAMI2015_Pruning_KCenter(PointCloud &ref, int k); // use a priority queue to keep K, the query is ordered by KCenter ordering
    void KNN_PAMI2015_Pruning_KCenter_orderdata(PointCloud &ref, int k); // order dataset first according to its size
    
    void KNN_PAMI2015_Pruning_KCenter_UB(PointCloud &ref, int k, int dataSizeThreshold=10); // using kcenter UB
    void KNN_PAMI2015_Pruning_KCenter_UB_BscLB(PointCloud &ref, int k, int dataSizeThreshold=100000, int dataSizeThresholdForBscLB=100000); // using kcenter UB， and BscLB
    // if BscLB == 0, do not use KCenter_UB
    void KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(PointCloud &ref, int k, int dataSizeThreshold=100000, int dataSizeThresholdForBscLB=100000); // using kcenter UB， and BscLB
    // sort data point cloud according to their size first
    void KNN_PAMI2015_Pruning_KCenter_UB_BscLB3(PointCloud &ref, int k, int dataSizeThreshold=100000, int dataSizeThresholdForBscLB=100000); // using kcenter UB， and BscLB
    
    // using hilbert order value as UB in inner loop
    void KNN_PAMI2015_Pruning_KCenter_HilbertUB(PointCloud &ref, int k, int dataSizeThreshold = 10000);
    void KNN_PAMI2015_Pruning_KCenter_HilbertOrder(PointCloud &ref, int k);
    
    void KNN_PAMI2015_Pruning_KCenter_MBR(PointCloud &ref, int k, int threshold = 100); //using kcenter and MBR(root mbr) to calculate lower bound.
    void KNN_PAMI2015_Pruning_KCenter_MBRs(PointCloud &ref, int k, int threshold = 100); //using kcenter and MBRs(ENHLB) to calculate lower bound.
    
    void KNN_PR2017_Pruning(PointCloud &ref, int k); // use a priority queue to keep K

    
    void KNN_COMBINED(PointCloud &ref, int k, int number = 10);
    void KNN_COMBINED_KCenter(PointCloud &ref, int k, int number = 10);
    // 错的！！！ 单边的hausdorff distance 不满足三角不等式！！！不要用这个method了！
    void KNN_COMBINED_KCenter_LB(PointCloud &ref, int k, int number = 10);
    // 但是 UB 仍然满足
    void KNN_COMBINED_KCenter_UB(PointCloud &ref, int k, int number = 10);
    void KNN_COMBINED_KCenter_UB2(PointCloud &ref, int k, int number = 10, int dataSizeThreshold=10);
    
    void KNN_MINE(PointCloud &ref, int k, double a, double b, int c, int querythreshold);
    void KNN_MINE2(PointCloud &ref, int k, int partialQuerySize);
    void KNN_MINE3(PointCloud &ref, int k, double percentage, int LB, int UB, int queryMBRNumber);
    void KNN_MINE4(PointCloud &ref, int k, double percentage, int LB, int UB, int queryMBRNumber); // using kcenter ordering
    void KNN_MINE5(PointCloud &ref, int k, double percentage, int LB, int UB, int queryMBRNumber, double KNNValue); // using kcenter ordering, using threshold in PartialHD
    void KNN_MINE6(PointCloud &ref, int k, int step, int queryMBRNumber, double percent=0.05, int lowerthreshold=20, int upperthreshold=100); // using kcenter ordering, and gradually improve progress
    void KNN_MINE7(PointCloud &ref, int k, int step, int queryMBRNumber, double percent=0.05, int lowerthreshold=20, int upperthreshold=100, int sizeThreshold=10000);
    void KNN_MINE7_2(PointCloud &ref, int k, int step, int queryMBRNumber, double percent=0.05, int lowerthreshold=20, int upperthreshold=100, int sizeThreshold=10000); // optimize queue ending
    void KNN_MINE7_3(PointCloud &ref, int k, int step, int queryMBRNumber, double percent=0.05, int lowerthreshold=20, int upperthreshold=100, int sizeThreshold=10000);// using bsclb to activate queue
    
    
    void KthValueRecord_KNN_PAMI2015_Pruning_KCenter(PointCloud &ref, int k);
    void Test_Time__KNN_PAMI2015_Pruning_KCenter(PointCloud &ref, int k);
    void Test_Time__KNN_PAMI2015_Pruning_KCenter2(PointCloud &ref, int k);
    void Test_Time_KNN_PAMI2015_Pruning(PointCloud &ref, int k);
    void Test_Time_KNN_PAMI2015_Pruning2(PointCloud &ref, int k);
    
    void AnalyseLBs(PointCloud &ref);
    
    void KNN_Center(PointCloud &ref, int k);
    void KNN_BHD(PointCloud &ref, int k);
    void KNN_HDLog(PointCloud &ref, int k); // HD(p,q)*log|p|
    
//    void NN_UsingPoint(PointCloud &ref); // currently do not support K, // need to bind call generateKeywordMap() AND buildRtreeForAllPoints() FIRST
//    void NN_UsingPoint_Efficient(PointCloud &ref); // 1.first sort dataset, then call build Rtree then generateBitmap
//    void KNN_UsingPoint_Efficient(PointCloud &ref, int k); // 1.first sort dataset, then call build Rtree then generateBitmap
    void KNN_UsingPoint_Efficient(PointCloud &ref, int k, map<int, vector<int>> &pidKeywordIdsMap, int randomNum = 1);
    
    void doIntersection(vector<vector<Point>> &candidateSets, vector<Point> &candidateSet);
};

bool cmp_pointcloudsize(PointCloud &pc1, PointCloud &pc2){
    return pc1.pointcloud.size() < pc2.pointcloud.size();
}

//KNNSearch::KNNSearch(string directory){
//    loadDataset(directory);
//}

void KNNSearch::randomizeDataset(){
    
    long size = this->dataset.size();
    int randomIndex = 0;
    srand((unsigned)time(NULL));
    PointCloud temp;
    for (int i = 0; i < size; i++){
        // create random variable in [0, size)
        randomIndex = rand()%size;
        // random exchange
        temp = dataset[i];
        dataset[i] = dataset[randomIndex];
        dataset[randomIndex] = temp;
    }
}

// http://www.martinbroadhurst.com/list-the-files-in-a-directory-in-c.html
vector<string> findFileInDir(string directory){
    vector<string> v;
    
    DIR* dirp = opendir(directory.c_str());
    struct dirent * dp;
    while ((dp = readdir(dirp)) != NULL) {
        v.push_back(dp->d_name); // just names
    }
    closedir(dirp);
    
    return v;
}

void KNNSearch::loadDataset(string directory){
    dataset.clear();
    vector<string> filenames = findFileInDir(directory);
    string filePath;
    for(int i = 0; i < filenames.size(); i++){
//        cout << filenames[i] << endl;
        filePath = directory+"/"+filenames[i];
        PointCloud pc = PointCloud(filePath);
        if(pc.pointcloud.size() == 0)
            continue;
        dataset.push_back(pc);
    }
}

// ascending order
bool cmp_hausdorff(pair<double,PointCloud> &p1, pair<double,PointCloud> &p2){
    return p1.first < p2.first;
}

vector<pair<double,PointCloud>> KNNSearch::KNN_PAMI2015(PointCloud &ref, int k){
    
    clock_t start,stop, time1, time2, time3, total;
    
    // preprocessing
    start = clock();
    ref.randomize();
    int count = 1;
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
        if (count % 100000 == 0)
            cout << count << endl;
        count++;
    }
    stop = clock();
    time1 = stop-start;
    
    // calculation
    start = clock();
    vector<pair<double,PointCloud>> all;
    double EHD = 0;
    count = 1;
    for(int i = 0; i < dataset.size(); i++){
        EHD = ExactHausdorff::PAMI2015(ref, dataset[i]);
        all.push_back(pair<double, PointCloud>(EHD, dataset[i]));
        if (count % 100000 == 0)
            cout << count << endl;
        count++;
    }
    stop = clock();
    time2 = stop-start;
    
    // sorting
    start = clock();
    sort(all.begin(),all.end(),cmp_hausdorff);
    vector<pair<double,PointCloud>> result(all.begin(), all.begin()+k);
    stop = clock();
    time3 = stop-start;
    
    // result
    total = time1+time2+time3;
    cout << "KNN_PAMI2015 K=" << k << " size=" << dataset.size() << " totaltime=" <<  total << " average=" << total/dataset.size() << endl;
    cout << " preprocessing=" << time1 << " calculation=" << time2 << " sorting=" << time3 << endl;
    
//    ofstream outfile;
//    outfile.open("/Users/lizhe/Desktop/record-PAMI2015", std::ios_base::app);
//    outfile << total << " " << time1 << " " << time2 << " " << time3 << endl;
//    outfile.close();
    
//    for(int i = 0; i < k; i++){
//        cout << result[i].first << endl;
//    }
    
    return result;
}

struct cmp_double{
    bool operator()(double d1, double d2){
        return d1 < d2;
    }
};

struct cmp_pair{
    bool operator()(pair<double,int> p1, pair<double,int> p2){
        return p1.first < p2.first;
    }
};

void KNNSearch::KNN_PAMI2015_Pruning(PointCloud &ref, int k){
    
    priority_queue<double, vector<double>, cmp_double> prqueue;
    
    ref.randomize();
    int count = 1;
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
//        if (count % 100000 == 0)
//            cout << count << endl;
//        count++;
    }
    
    clock_t start, stop;
    start = clock();
    
    double distance;
    count = 1;
    long size1 = ref.pointcloud.size();
    double max = 0;
    double min = 0;
    double kthValue = 0;
    bool readyTag = false;
    bool breakTag = false;
    for(int i = 0; i < dataset.size(); i++){
        long size2 = dataset[i].pointcloud.size();
        max = 0;
        breakTag = false;
        // calculation of Hausdorff distance
        for (int j = 0; j < size1; j++){
            min = std::numeric_limits<double>::infinity();
            for(int m = 0; m < size2; m++){
                distance = ref.pointcloud[j].distanceTo(dataset[i].pointcloud[m]);
                if(distance < min){
                    min = distance;
                }
                if(distance <= max){
                    break;
                }
            }
            if(min > max){
                max = min;
                if(readyTag){
                    if(max >= kthValue){
                        breakTag = true;
                        break;
                    }
                }
            }
        }
        
//        if(fabs(max - 1.6393) < 0.0001){
//            cout << "here" << endl;
//        }
        
        if(breakTag){
            continue;
        }
        
        if(prqueue.size() < k){
            prqueue.push(max);
            if(prqueue.size() == k){
                readyTag = true;
                kthValue = prqueue.top(); // 这不是第K 个 吧 ？？？？？？？？？
            }
        } else{
            // the new max must less than kthValue, or it will be prune
            prqueue.pop();
            prqueue.push(max);
            kthValue = prqueue.top();
        }
        
//        if (count % 100000 == 0)
//            cout << count << endl;
        count++;
    }
    stop = clock();
    cout << "PAMI2015_pruning time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout<< prqueue.top() << endl;
        prqueue.pop();
    }
}

void KNNSearch::KNN_PAMI2015_Pruning_KCenter(PointCloud &ref, int k){
//    priority_queue<double, vector<double>, cmp_double> prqueue;
    priority_queue<pair<double, int>, vector<pair<double, int>>, cmp_pair> prqueue;
    
    ref.sortByKcenter();
//    ref.randomize();
    int count = 1;
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
        //        if (count % 100000 == 0)
        //            cout << count << endl;
        //        count++;
    }
    
    clock_t start, stop;
    start = clock();
    
    double distance;
    count = 1;
    long size1 = ref.pointcloud.size();
    double max = 0;
    double min = 0;
    double kthValue = 0;
    bool readyTag = false;
    bool breakTag = false;
    for(int i = 0; i < dataset.size(); i++){
        long size2 = dataset[i].pointcloud.size();
        max = 0;
        breakTag = false;
        // calculation of Hausdorff distance
        for (int j = 0; j < size1; j++){
            min = std::numeric_limits<double>::infinity();
            for(int m = 0; m < size2; m++){
                distance = ref.pointcloud[j].distanceTo(dataset[i].pointcloud[m]);
                if(distance < min){
                    min = distance;
                }
                if(distance <= max){
                    break;
                }
            }
            if(min > max){
                max = min;
                if(readyTag){
                    if(max >= kthValue){
                        breakTag = true;
                        break;
                    }
                }
            }
        }
        
        //        if(fabs(max - 1.6393) < 0.0001){
        //            cout << "here" << endl;
        //        }
        
        if(breakTag){
            continue;
        }
        
        if(prqueue.size() < k){
//            prqueue.push(max);
            prqueue.push(pair<double,int>(max,i));
            if(prqueue.size() == k){
                readyTag = true;
//                kthValue = prqueue.top(); // 这不是第K 个 吧 ？？？？？？？？？
                kthValue = prqueue.top().first;
            }
        } else{
            // the new max must less than kthValue, or it will be prune
            prqueue.pop();
//            prqueue.push(max);
            prqueue.push(pair<double,int>(max,i));
//            kthValue = prqueue.top();
            kthValue = prqueue.top().first;
        }
        
        //        if (count % 100000 == 0)
        //            cout << count << endl;
        count++;
    }
    stop = clock();
    
    ofstream outfile;
    outfile.open("/Users/lizhe/Desktop/reports/final_01/using_HD.csv", ofstream::app);
    outfile << endl;
    
    cout << "PAMI2015_pruning_kcenter time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << dataset[prqueue.top().second].keywordId << "\t\tdistance: " << prqueue.top().first << "\t\tsize: "<< dataset[prqueue.top().second].pointcloud.size() << endl;
        outfile << dataset[prqueue.top().second].keywordId << "," << prqueue.top().first << ","<< dataset[prqueue.top().second].pointcloud.size() << endl;
        prqueue.pop();
    }
}

void KNNSearch::KNN_PAMI2015_Pruning_KCenter_orderdata(PointCloud &ref, int k){
    priority_queue<double, vector<double>, cmp_double> prqueue;
    
    ref.sortByKcenter();
    //    ref.randomize();
    int count = 1;
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
        //        if (count % 100000 == 0)
        //            cout << count << endl;
        //        count++;
    }
    sort(dataset.begin(), dataset.end(), cmp_pointcloudsize);
    
    clock_t start, stop;
    start = clock();
    
    double distance;
    count = 1;
    long size1 = ref.pointcloud.size();
    double max = 0;
    double min = 0;
    double kthValue = 0;
    bool readyTag = false;
    bool breakTag = false;
    for(int i = 0; i < dataset.size(); i++){
        long size2 = dataset[i].pointcloud.size();
        max = 0;
        breakTag = false;
        // calculation of Hausdorff distance
        for (int j = 0; j < size1; j++){
            min = std::numeric_limits<double>::infinity();
            for(int m = 0; m < size2; m++){
                distance = ref.pointcloud[j].distanceTo(dataset[i].pointcloud[m]);
                if(distance < min){
                    min = distance;
                }
                if(distance <= max){
                    break;
                }
            }
            if(min > max){
                max = min;
                if(readyTag){
                    if(max >= kthValue){
                        breakTag = true;
                        break;
                    }
                }
            }
        }
        
        //        if(fabs(max - 1.6393) < 0.0001){
        //            cout << "here" << endl;
        //        }
        
        if(breakTag){
            continue;
        }
        
        if(prqueue.size() < k){
            prqueue.push(max);
            if(prqueue.size() == k){
                readyTag = true;
                kthValue = prqueue.top(); // 这不是第K 个 吧 ？？？？？？？？？
            }
        } else{
            // the new max must less than kthValue, or it will be prune
            prqueue.pop();
            prqueue.push(max);
            kthValue = prqueue.top();
        }
        
        //        if (count % 100000 == 0)
        //            cout << count << endl;
        count++;
    }
    stop = clock();
    cout << "PAMI2015_pruning_kcenter order dataset first time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout<< prqueue.top() << endl;
        prqueue.pop();
    }
}

// using kcenter UB
void KNNSearch::KNN_PAMI2015_Pruning_KCenter_UB(PointCloud &ref, int k, int dataSizeThreshold){
    
    vector<pair<double,int>> disToKcenter = ref.sortByKcenterWithRecord();
    
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    priority_queue<double> krecord; // descending
    
    clock_t start, stop;
    start = clock();
    
    double distance = 0;
    double kthValue = std::numeric_limits<double>::infinity();
    
    for(int i = 0; i < dataset.size(); i++){
        if(dataset[i].pointcloud.size() < dataSizeThreshold){
            distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        } else{
            distance = ExactHausdorff::PAMI2015_Pruning_KCenterUB(ref, dataset[i], disToKcenter, kthValue);
        }
        if(distance == -1){
            continue;
        } else if(krecord.size() < k){
            krecord.push(distance);
        } else if(distance < kthValue){
            krecord.pop();
            krecord.push(distance);
            kthValue = krecord.top();
        }
    }
    
    stop = clock();
    
    cout << "PAMI kcenter UB method time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

void KNNSearch::KNN_PAMI2015_Pruning_KCenter_UB_BscLB(PointCloud &ref, int k, int dataSizeThreshold, int dataSizeThresholdForBscLB){
    
    ref.generateBoundAndMBRs(10);
    vector<pair<double,int>> disToKcenter = ref.sortByKcenterWithRecord();
    
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    priority_queue<double> krecord; // descending
    
    clock_t start, stop;
    start = clock();
    
    double distance = 0;
    double kthValue = std::numeric_limits<double>::infinity();
    
    for(int i = 0; i < dataset.size(); i++){
        // handle bound
        if(krecord.size() == k && dataset[i].pointcloud.size() > dataSizeThresholdForBscLB){
            distance = HausdorffDistanceForBound(ref.bound, dataset[i].bound);
            if(distance > kthValue){
                continue;
            }
        }
        // handle exact
        if(dataset[i].pointcloud.size() < dataSizeThreshold){
            distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        } else{
            distance = ExactHausdorff::PAMI2015_Pruning_KCenterUB(ref, dataset[i], disToKcenter, kthValue);
        }
        if(distance == -1){
            continue;
        } else if(krecord.size() < k){
            krecord.push(distance);
        } else if(distance < kthValue){
            krecord.pop();
            krecord.push(distance);
            kthValue = krecord.top();
        }
    }
    
    stop = clock();
    
    cout << "PAMI kcenter UB with BscLB method time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

// if BscLB == 0, do not use KCenterUB
void KNNSearch::KNN_PAMI2015_Pruning_KCenter_UB_BscLB2(PointCloud &ref, int k, int dataSizeThreshold, int dataSizeThresholdForBscLB){
    ref.generateBoundAndMBRs(10);
    vector<pair<double,int>> disToKcenter = ref.sortByKcenterWithRecord();
    
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    priority_queue<double> krecord; // descending
    
    clock_t start, stop;
    start = clock();
    
    double distance = 0;
    double kthValue = std::numeric_limits<double>::infinity();
    bool KCenterUB = true;
    
    for(int i = 0; i < dataset.size(); i++){
        
        KCenterUB = true;
        
        // handle bound
        if(krecord.size() == k && dataset[i].pointcloud.size() > dataSizeThresholdForBscLB){
            distance = HausdorffDistanceForBound(ref.bound, dataset[i].bound);
            if(distance > kthValue){
                continue;
            } else if(distance == 0){
                KCenterUB = false;
            }
        }
        // handle exact
        if(dataset[i].pointcloud.size() < dataSizeThreshold || !KCenterUB){
            distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        } else {
            distance = ExactHausdorff::PAMI2015_Pruning_KCenterUB(ref, dataset[i], disToKcenter, kthValue);
        }
        if(distance == -1){
            continue;
        } else if(krecord.size() < k){
            krecord.push(distance);
        } else if(distance < krecord.top()){
            krecord.pop();
            krecord.push(distance);
            kthValue = krecord.top();
        }
        // remove this later
//        cout << distance << endl;
    }
    
    stop = clock();
    
    cout << "PAMI kcenter UB with BscLB 2 method time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

// sort data point cloud according to their size first
void KNNSearch::KNN_PAMI2015_Pruning_KCenter_UB_BscLB3(PointCloud &ref, int k, int dataSizeThreshold, int dataSizeThresholdForBscLB){
    ref.generateBoundAndMBRs(10);
    vector<pair<double,int>> disToKcenter = ref.sortByKcenterWithRecord();
    
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    sort(dataset.begin(), dataset.end(), cmp_pointcloudsize);
    priority_queue<double> krecord; // descending
    
    clock_t start, stop;
    start = clock();
    
    double distance = 0;
    double kthValue = std::numeric_limits<double>::infinity();
    bool KCenterUB = true;
    
    for(int i = 0; i < dataset.size(); i++){
        
        KCenterUB = true;
        
        // handle bound
        if(krecord.size() == k && dataset[i].pointcloud.size() > dataSizeThresholdForBscLB){
            distance = HausdorffDistanceForBound(ref.bound, dataset[i].bound);
            if(distance > kthValue){
                continue;
            } else if(distance == 0){
                KCenterUB = false;
            }
        }
        // handle exact
        if(dataset[i].pointcloud.size() < dataSizeThreshold || !KCenterUB){
            distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        } else {
            distance = ExactHausdorff::PAMI2015_Pruning_KCenterUB(ref, dataset[i], disToKcenter, kthValue);
        }
        if(distance == -1){
            continue;
        } else if(krecord.size() < k){
            krecord.push(distance);
        } else if(distance < kthValue){
            krecord.pop();
            krecord.push(distance);
            kthValue = krecord.top();
        }
    }
    
    stop = clock();
    
    cout << "PAMI kcenter UB with BscLB 3 method time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

//bool cmp_pointcloudsize(PointCloud &pc1, PointCloud &pc2){
//    return pc1.pointcloud.size() < pc2.pointcloud.size();
//}

void KNNSearch::KNN_PAMI2015_Pruning_KCenter_HilbertUB(PointCloud &ref, int k, int dataSizeThreshold){
    
    ref.sortByKcenter();
    ref.calculateHilbertValue();
    
    // dataset.order by hilbert value
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].calculateHilbertValue();
        dataset[i].myHilbertOrder();
//        if(i % 10000 == 0){
//            cout << "Hilbert preparement... " << i << endl;
//        }
    }
    cout << "finish hilbert preparement..." << endl;
    
    priority_queue<double> krecord; // descending
    
    clock_t start, stop;
    start = clock();
    
    double distance = 0;
    double kthValue = std::numeric_limits<double>::infinity();
    
    for(int i = 0; i < dataset.size(); i++){
                
        if(dataset[i].pointcloud.size() < dataSizeThreshold){
            distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        }
        else{
            distance = ExactHausdorff::PAMI2015_UsingHilbert(ref, dataset[i], true, kthValue);
//            distance = ExactHausdorff::PAMI2015_UsingHilbert(ref, dataset[i]);
        }
        if(distance == -1){
            continue;
        } else if(krecord.size() < k){
            krecord.push(distance);
        } else if(distance < kthValue){
            krecord.pop();
            krecord.push(distance);
            kthValue = krecord.top();
        }
        
        // remove this later
//        cout << distance << endl;
    }
    
    stop = clock();
    
    cout << "PAMI kcenter Hilbert UB method time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

void KNNSearch::KNN_PAMI2015_Pruning_KCenter_HilbertOrder(PointCloud &ref, int k){
    ref.sortByKcenter();
//    ref.calculateHilbertValue();
    // dataset.order by hilbert value
    for(int i = 0; i < dataset.size(); i++){
//        dataset[i].calculateHilbertValue();
//        dataset[i].myHilbertOrder();
        dataset[i].hilbertOrder();
    }
    cout << "finish hilbert preparement..." << endl;
    
    priority_queue<double> krecord; // descending
    
    clock_t start, stop;
    start = clock();
    
    double distance = 0;
    double kthValue = std::numeric_limits<double>::infinity();
    
    for(int i = 0; i < dataset.size(); i++){

        distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        if(distance == -1){
            continue;
        } else if(krecord.size() < k){
            krecord.push(distance);
        } else if(distance < kthValue){
            krecord.pop();
            krecord.push(distance);
            kthValue = krecord.top();
        }
    }
    
    stop = clock();
    
    cout << "PAMI kcenter Hilbert Order method time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

void KNNSearch::Test_Time__KNN_PAMI2015_Pruning_KCenter(PointCloud &ref, int k){
    
    ref.sortByKcenter();
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    sort(dataset.begin(), dataset.end(), cmp_pointcloudsize);
    long tenPercent = dataset.size()/10;
    
    priority_queue<double> krecord; // descending
    
    clock_t start, stop, total=0;
    start = clock();
    
    double distance = 0;
    double kthValue = std::numeric_limits<double>::infinity();
    
    for(int i = 0; i < dataset.size(); i++){
        
        if(i % tenPercent == 0){ // if do this, some remainning point clouds do not count,
            stop = clock();
            cout << dataset[i].pointcloud.size() << " \t " << stop-start << endl;
            total += stop-start;
            start = clock();
        }
        
        
        distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        if(distance == -1){
            continue;
        } else if(krecord.size() < k){
            krecord.push(distance);
        } else if(distance < kthValue){
            krecord.pop();
            krecord.push(distance);
            kthValue = krecord.top();
        }
    }
    
    stop = clock();
    
    cout << "PAMI kcenter last few records time usage: " << stop-start << endl;
    total += stop-start;
    cout << "total time usage: " << total << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

void KNNSearch::Test_Time__KNN_PAMI2015_Pruning_KCenter2(PointCloud &ref, int k){
    
    ref.sortByKcenter();
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    sort(dataset.begin(), dataset.end(), cmp_pointcloudsize);
    priority_queue<double> krecord; // descending
    
    clock_t start, stop, total=0;
    start = clock();
    
    double distance = 0;
    double kthValue = std::numeric_limits<double>::infinity();
    bool firstflag1 = true, firstflag2 = true, firstflag3 = true, firstflag4 = true, firstflag5 = true;
    int count0=0, count1=0, count2=0, count3=0, count4=0, count5=0;
    
    for(int i = 0; i < dataset.size(); i++){
        long sizedata = dataset[i].pointcloud.size();
        if(sizedata < 10){
            // do noting
            count0++;
        }else if(sizedata < 99){
            if(firstflag1){
                stop = clock();
                cout << "1 - 10: " << stop-start << "   count: " << count0 << endl;
                total += stop-start;
                start = clock();
                firstflag1 = false;
            }
            count1++;
        }else if(sizedata < 999){
            if(firstflag2){
                stop = clock();
                cout << "10 - 99: " << stop-start << "   count: " << count1 << endl;
                total += stop-start;
                start = clock();
                firstflag2 = false;
            }
            count2++;
        }else if(sizedata < 9999){
            if(firstflag3){
                stop = clock();
                cout << "100 - 999: " << stop-start << "   count: " << count2 << endl;
                total += stop-start;
                start = clock();
                firstflag3 = false;
            }
            count3++;
        }else if(sizedata < 99999){
            if(firstflag4){
                stop = clock();
                cout << "1000 - 9999: " << stop-start << "   count: " << count3 <<endl;
                total += stop-start;
                start = clock();
                firstflag4 = false;
            }
            count4++;
        }else if(sizedata < 999999){
            if(firstflag5){
                stop = clock();
                cout << "10,000 - 99,999: " << stop-start << "   count: " << count4 << endl;
                total += stop-start;
                start = clock();
                firstflag5 = false;
            }
            count5++;
        }
        
        distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        if(distance == -1){
            continue;
        } else if(krecord.size() < k){
            krecord.push(distance);
        } else if(distance < kthValue){
            krecord.pop();
            krecord.push(distance);
            kthValue = krecord.top();
        }
    }
    
    stop = clock();
    
    cout << "100,000 - 990,999: " << stop-start << "   count: " << count5 <<endl;
    total += stop-start;
    cout << "PAMI testing time 2 total time usage: " << total << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

void KNNSearch::KthValueRecord_KNN_PAMI2015_Pruning_KCenter(PointCloud &ref, int k){
    vector<pair<double,int>> disToKcenter = ref.sortByKcenterWithRecord();
    
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    priority_queue<double> krecord; // descending
    
    clock_t start, stop;
    start = clock();
    
    double distance = 0;
    double kthValue = std::numeric_limits<double>::infinity();
    
    int largePointCloudCount = 0;
    int allPointCloudCount = 0;
    
    ofstream outfile1, outfile2;
    outfile1.open("/Users/lizhe/Desktop/reports/KNN9/data1");
    outfile2.open("/Users/lizhe/Desktop/reports/KNN9/data2");
    
    for(int i = 0; i < dataset.size(); i++){
        distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        if(distance == -1){
            continue;
        } else if(krecord.size() < k){
            krecord.push(distance);
        } else if(distance < kthValue){
            krecord.pop();
            krecord.push(distance);
            kthValue = krecord.top();
            
            allPointCloudCount++;
            outfile2 << allPointCloudCount << " " << kthValue << endl;
            if(dataset[i].pointcloud.size() >= 10000){
//                largePointCloudCount++;
//                outfile1 << largePointCloudCount << " " << kthValue << endl;
                outfile1 << allPointCloudCount << " " << kthValue << endl;
            }
        }
    }
    
    stop = clock();
    
    cout << "PAMI kcenter UB method time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

void KNNSearch::Test_Time_KNN_PAMI2015_Pruning(PointCloud &ref, int k){
    
    ref.randomize();
    int count = 1;
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
        if (count % 100000 == 0)
            cout << count << endl;
        count++;
    }
    
    sort(dataset.begin(), dataset.end(), cmp_pointcloudsize);
    long tenPercent = dataset.size()/10;
    long size;
    double distance;
    double kthValue = 0;
    priority_queue<double, vector<double>, cmp_double> prqueue;
    bool pruning = false;
    clock_t start, stop, total=0, start2, stop2,total2 = 0;
    start = clock();
    start2 = clock();
    for(int i = 1; i <= dataset.size(); i++){
        
        if(prqueue.size() == k){
            pruning = true;
            kthValue = prqueue.top();
        }
        
        // calculation single distance
        distance = ExactHausdorff::PAMI2015(ref, dataset[i], pruning, kthValue);
        prqueue.push(distance);
        
        if(i % tenPercent == 0){ // if do this, some remainning point clouds do not count,
            stop = clock();
            cout << dataset[i].pointcloud.size() << " \t " << (stop-start)/1000 << endl;
            total += stop-start;
            start = clock();
        }
    }
    stop = clock();
    cout << "remaining time spent: " << (stop-start)/1000 << endl;
    stop2 = clock();
    total2 = stop2 - start2;
    cout << "total time: " << total << endl;
    cout << "total time 2: " << total2/1000 << endl;
}

void KNNSearch::Test_Time_KNN_PAMI2015_Pruning2(PointCloud &ref, int k){
    ref.randomize();
    int count = 1;
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
        if (count % 100000 == 0)
            cout << count << endl;
        count++;
    }
    
    sort(dataset.begin(), dataset.end(), cmp_pointcloudsize);
    long tenPercent = dataset.size()/10;
    long size;
    double distance;
    double kthValue = 0;
    priority_queue<double, vector<double>, cmp_double> prqueue;
    bool pruning = false;
    clock_t start, stop;
    start = clock();
    count = 0;
    int standard = 100;
    int range = 0;
    for(int i = 1; i <= dataset.size(); i++){
        
        count++;
        
        if(int(dataset[i].pointcloud.size() / standard) > range){
            stop = clock();
            cout << range << " " << (stop-start)/(count*1000) << endl;
            count = 0;
            range = dataset[i].pointcloud.size() / standard;
            start = clock();
        }
        
        if(prqueue.size() == k){
            pruning = true;
            kthValue = prqueue.top();
        }
        
        // calculation single distance
        distance = ExactHausdorff::PAMI2015(ref, dataset[i], pruning, kthValue);
        prqueue.push(distance);
        count++;
        
    }
}

vector<pair<double,PointCloud>> KNNSearch::KNN_PR2017(PointCloud &ref, int k){

    clock_t start,stop, time1, time2, time3, total;
    
    // preprcessing
    start = clock();
    ref.prezorder();
    ref.zorder();
    int count = 1;
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].prezorder();
        dataset[i].zorder();
        if (count % 100000 == 0)
            cout << count << endl;
        count++;
    }
    stop = clock();
    time1 = stop-start;
    
    // calculation
    start = clock();
    vector<pair<double,PointCloud>> all;
    double EHD = 0;
    count = 1;
    for(int i = 0; i < dataset.size(); i++){
        EHD = ExactHausdorff::PR2017(ref, dataset[i]);
        all.push_back(pair<double, PointCloud>(EHD, dataset[i]));
        if (count % 100000 == 0)
            cout << count << endl;
        count++;
    }
    stop = clock();
    time2 = stop-start;
    
    // sorting
    start = clock();
    sort(all.begin(),all.end(),cmp_hausdorff);
    vector<pair<double,PointCloud>> result(all.begin(), all.begin()+k);
    stop = clock();
    time3 = stop-start;
    
    // result
    total = time1+time2+time3;
    cout << "KNN_PR2017 K=" << k << " size=" << dataset.size() << " totaltime=" <<  total << " average=" << total/dataset.size() << endl;
    cout << " preprocessing=" << time1 << " calculation=" << time2 << " sorting=" << time3 << endl;
//    ofstream outfile;
//    outfile.open("/Users/lizhe/Desktop/record-PR2017", std::ios_base::app);
//    outfile << total << " " << time1 << " " << time2 << " " << time3 << endl;
//    outfile.close();
    
    return result;
}

void KNNSearch::KNN_PAMI2015_Pruning_KCenter_MBR(PointCloud &ref, int k, int threshold){
    priority_queue<pair<double, int>, vector<pair<double, int>>, cmp_pair> prqueue;
    ref.calculateCenterPoint();

    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    clock_t start, stop;
    start = clock();
    
    double kthValue = std::numeric_limits<double>::infinity();
    double distance = 0;
    double lowerbound = 0;
    double bsclb = 0;
    
    for(int i = 0; i < dataset.size(); i++){
        bsclb = HausdorffDistanceForBound(ref.bound, dataset[i].bound);
        if(dataset[i].pointcloud.size() > threshold && bsclb != 0){
            lowerbound = ExactHausdorff::LowerboundFromKCenterToBound(ref,dataset[i].bound,kthValue);
            if(lowerbound >= kthValue){
                continue;
            }
        }
        distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        if(distance == -1){
            continue;
        }
        if(prqueue.size() < k){
            prqueue.push(pair<double, int>(distance, i));
        } else if(distance < prqueue.top().first){
            prqueue.pop();
            prqueue.push(pair<double, int>(distance, i));
            kthValue = prqueue.top().first;
        }
    }
    
    stop = clock();
    
    ofstream outfile;
    outfile.open("/Users/lizhe/Desktop/reports/final_01/using_Center.csv", ofstream::app);
    outfile << endl;
    
    cout << "using kcenter+MBR for LB" << " time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << dataset[prqueue.top().second].keyword << "\t\tdistance: " << prqueue.top().first << "\t\tsize: "<< dataset[prqueue.top().second].pointcloud.size() << endl;
        outfile << dataset[prqueue.top().second].keyword << "," << prqueue.top().first << ","<< dataset[prqueue.top().second].pointcloud.size() << endl;
        prqueue.pop();
    }
}



void KNNSearch::KNN_PAMI2015_Pruning_KCenter_MBRs(PointCloud &ref, int k, int threshold){
    priority_queue<pair<double, int>, vector<pair<double, int>>, cmp_pair> prqueue;
    ref.calculateCenterPoint();
    
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    clock_t start, stop;
    start = clock();
    
    double kthValue = std::numeric_limits<double>::infinity();
    double distance = 0;
    double lowerbound = 0;
    double bsclb = 0;
    
    for(int i = 0; i < dataset.size(); i++){
        bsclb = HausdorffDistanceForBound(ref.bound, dataset[i].bound);
        if(dataset[i].pointcloud.size() > threshold && bsclb != 0){
//            lowerbound = ExactHausdorff::LowerboundFromKCenterToBound(ref,dataset[i].bound,kthValue);
            lowerbound = ExactHausdorff::LowerboundFromKCenterToENHLB(ref, dataset[i].FirstLevelMBRs, kthValue);
            if(lowerbound >= kthValue){
                continue;
            }
        }
        distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        if(distance == -1){
            continue;
        }
        if(prqueue.size() < k){
            prqueue.push(pair<double, int>(distance, i));
        } else if(distance < prqueue.top().first){
            prqueue.pop();
            prqueue.push(pair<double, int>(distance, i));
            kthValue = prqueue.top().first;
        }
    }
    
    stop = clock();
    
    ofstream outfile;
    outfile.open("/Users/lizhe/Desktop/reports/final_01/using_Center.csv", ofstream::app);
    outfile << endl;
    
    cout << "using kcenter+MBRs for LB" << " time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << dataset[prqueue.top().second].keyword << "\t\tdistance: " << prqueue.top().first << "\t\tsize: "<< dataset[prqueue.top().second].pointcloud.size() << endl;
        outfile << dataset[prqueue.top().second].keyword << "," << prqueue.top().first << ","<< dataset[prqueue.top().second].pointcloud.size() << endl;
        prqueue.pop();
    }
}

void KNNSearch::KNN_PR2017_Pruning(PointCloud &ref, int k){
    clock_t start, stop;
    start = clock();
    priority_queue<double, vector<double>, cmp_double> prqueue;
    
    ref.prezorder();
    ref.zorder();
    int count = 1;
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].prezorder();
        dataset[i].zorder();
        if (count % 100000 == 0)
            cout << count << endl;
        count++;
    }
    
    double distance;
    count = 1;
    long size1 = ref.pointcloud.size();
    double max = 0;
    double min = 0;
    double kthValue = 0;
    bool readyTag = false;
    bool breakTag = false;
    int breakindex = 0;
    for(int i = 0; i < dataset.size(); i++){
        long size2 = dataset[i].pointcloud.size();
        max = 0;
        breakTag = false;
        // calculation of Hausdorff distance
        breakindex = 0;
        for (int j = 0; j < size1; j++){
            min = std::numeric_limits<double>::infinity();
            for(int m = 0; breakindex+m < size2 || breakindex-m > 0 ; m++){
                if (breakindex+m < size2){
                    distance = ref.pointcloud[j].distanceTo(dataset[i].pointcloud[breakindex+m]);
                    if(distance < min){
                        min = distance;
                    }
                    if(distance < max){
                        breakindex = breakindex+m;
                        break;
                    }
                }
                if (breakindex-m > 0){
                    distance = ref.pointcloud[j].distanceTo(dataset[i].pointcloud[breakindex-m]);
                    if(distance < min){
                        min = distance;
                    }
                    if(distance < max){
                        breakindex = breakindex-m;
                        break;
                    }
                }
            }
            if(min > max){
                max = min;
                if(readyTag){
                    if(max >= kthValue){
                        breakTag = true;
                        break;
                    }
                }
            }
        }
        
        if(breakTag){
            continue;
        }
        
        if(prqueue.size() < k){
            prqueue.push(max);
            if(prqueue.size() == k){
                readyTag = true;
                kthValue = prqueue.top();
            }
        }
        
        // the new max must less than kthValue, or it will be prune
        prqueue.pop();
        prqueue.push(max);
        kthValue = prqueue.top();
        
        if (count % 100000 == 0)
            cout << count << endl;
        count++;
    }
    stop = clock();
    cout << "PR2017 time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout<< prqueue.top() << endl;
        prqueue.pop();
    }
}

struct LBNode{
    double bsclb;
    double enhlb;
    double partialHD;
    double exactHD;
};

bool cmp_LBNode_bsclb(LBNode &n1, LBNode &n2){
    return n1.bsclb < n2.bsclb;
}

bool cmp_LBNode_enhlb(LBNode &n1, LBNode &n2){
    return n1.enhlb < n2.enhlb;
}

bool cmp_LBNode_partialHD(LBNode &n1, LBNode &n2){
    return n1.partialHD < n2.partialHD;
}

bool cmp_LBNode_exactHD(LBNode &n1, LBNode &n2){
    return n1.exactHD < n2.exactHD;
}

void KNNSearch::AnalyseLBs(PointCloud &ref){
    rtree refRTree(ref.pointcloud.begin(), ref.pointcloud.end());
    int number = 10;
    
    // calculate ref's bound
    RTV::box_type refbound = refRTree.bounds();
    Point p1(refbound.m_min_corner.m_values[0], refbound.m_min_corner.m_values[1]);
    p1.dimension = 2;
    Point p2(refbound.m_max_corner.m_values[0], refbound.m_max_corner.m_values[1]);
    p2.dimension = 2;
    ref.bound = pair<Point, Point>(p1, p2);
    
    // calculate ref's MBRs
    vector<RTV::box_type> refVec = getMBRs2(refRTree, number); // extract the first level
    vector<pair<Point, Point>> refMBRs;
    for(int i = 0; i < refVec.size(); i++){
        Point p1(refVec[i].m_min_corner.m_values[0], refVec[i].m_min_corner.m_values[1]);
        p1.dimension = 2;
        Point p2(refVec[i].m_max_corner.m_values[0], refVec[i].m_max_corner.m_values[1]);
        p2.dimension = 2;
        refMBRs.push_back(pair<Point, Point>(p1, p2));
    }
    // if there are no first level MBRs
    if(refMBRs.size() == 0){
        refMBRs.push_back(pair<Point, Point>(ref.bound));
    }
    ref.FirstLevelMBRs = refMBRs;
    
    // do randomize for both query and data
//    ref.randomize();
//    for(int i = 0; i < dataset.size(); i++){
//        dataset[i].randomize();
//    }
    
    // do KCenter ordering
    ref.sortByKcenter();
    
    vector<double> bsclb;
    vector<double> enhlb;
    vector<double> partialHD;
    vector<double> exactHD;
    vector<LBNode> nodes;
    
    double distance1 = 0;
    double distance2 = 0;
    double distance3 = 0;
    double distance4 = 0;
    int progress = ref.pointcloud.size() * 0.05;
    for(int i = 0; i < dataset.size(); i++){
        LBNode node;
        distance1 = HausdorffDistanceForBound(ref.bound, dataset[i].bound);
        distance2 = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[i].FirstLevelMBRs);
        distance3 = ExactHausdorff::Partial_PAMI2015(ref, dataset[i], progress);
        distance4 = ExactHausdorff::PAMI2015(ref, dataset[i]);
        
        bsclb.push_back(distance1);
        enhlb.push_back(distance2);
        partialHD.push_back(distance3);
        exactHD.push_back(distance4);
        
        node.bsclb = distance1;
        node.enhlb = distance2;
        node.partialHD = distance3;
        node.exactHD = distance4;
        nodes.push_back(node);
        cout << "calculating..." << i << endl;
    }
    
    sort(bsclb.begin(), bsclb.end());
    sort(enhlb.begin(), enhlb.end());
    sort(partialHD.begin(), partialHD.end());
    sort(exactHD.begin(), exactHD.end());
    
    ofstream outfile1, outfile2, outfile3, outfile4;
    outfile1.open("/Users/lizhe/Desktop/reports/KNN8/kcenter-sort-bsclb");
    outfile2.open("/Users/lizhe/Desktop/reports/KNN8/kcenter-sort-enhlb");
    outfile3.open("/Users/lizhe/Desktop/reports/KNN8/kcenter-sort-partialHD");
    outfile4.open("/Users/lizhe/Desktop/reports/KNN8/kcenter-sort-exactHD");
    
    for(int i = 0; i < dataset.size(); i++){
        outfile1 << i << " " << bsclb[i] << endl;
        outfile2 << i << " " << enhlb[i] << endl;
        outfile3 << i << " " << partialHD[i] << endl;
        outfile4 << i << " " << exactHD[i] << endl;
    }
    outfile1.close();
    outfile2.close();
    outfile3.close();
    outfile4.close();
    
    ofstream outfile_node1, outfile_node2, outfile_node3, outfile_node4;
    outfile_node1.open("/Users/lizhe/Desktop/reports/KNN8/kcenter-sort-bsclb-node");
    sort(nodes.begin(), nodes.end(), cmp_LBNode_bsclb);
    for(int i = 0; i < dataset.size(); i++){
        outfile_node1 << i << " " << nodes[i].bsclb << " " << nodes[i].enhlb << " " << nodes[i].partialHD << " " << nodes[i].exactHD << endl;
    }
    outfile_node1.close();

    outfile_node2.open("/Users/lizhe/Desktop/reports/KNN8/kcenter-sort-enhlb-node");
    sort(nodes.begin(), nodes.end(), cmp_LBNode_enhlb);
    for(int i = 0; i < dataset.size(); i++){
        outfile_node2 << i << " " << nodes[i].bsclb << " " << nodes[i].enhlb << " " << nodes[i].partialHD << " " << nodes[i].exactHD << endl;
    }
    outfile_node2.close();

    outfile_node3.open("/Users/lizhe/Desktop/reports/KNN8/kcenter-sort-partialHD-node");
    sort(nodes.begin(), nodes.end(), cmp_LBNode_partialHD);
    for(int i = 0; i < dataset.size(); i++){
        outfile_node3 << i << " " << nodes[i].bsclb << " " << nodes[i].enhlb << " " << nodes[i].partialHD << " " << nodes[i].exactHD << endl;
    }
    outfile_node3.close();

    outfile_node4.open("/Users/lizhe/Desktop/reports/KNN8/kcenter-sort-exactHD-node");
    sort(nodes.begin(), nodes.end(), cmp_LBNode_exactHD);
    for(int i = 0; i < dataset.size(); i++){
        outfile_node4 << i << " " << nodes[i].bsclb << " " << nodes[i].enhlb << " " << nodes[i].partialHD << " " << nodes[i].exactHD << endl;
    }
    outfile_node4.close();
}

// ====================== a combined method ==========================
void KNNSearch::KNN_COMBINED(PointCloud &ref, int k, int number){
    
    rtree refRTree(ref.pointcloud.begin(), ref.pointcloud.end());
    
    // calculate ref's bound
    RTV::box_type refbound = refRTree.bounds();
    Point p1(refbound.m_min_corner.m_values[0], refbound.m_min_corner.m_values[1]);
    p1.dimension = 2;
    Point p2(refbound.m_max_corner.m_values[0], refbound.m_max_corner.m_values[1]);
    p2.dimension = 2;
    ref.bound = pair<Point, Point>(p1, p2);
    
    // calculate ref's MBRs
    vector<RTV::box_type> refVec = getMBRs2(refRTree, number); // extract the first level
    vector<pair<Point, Point>> refMBRs;
    for(int i = 0; i < refVec.size(); i++){
        Point p1(refVec[i].m_min_corner.m_values[0], refVec[i].m_min_corner.m_values[1]);
        p1.dimension = 2;
        Point p2(refVec[i].m_max_corner.m_values[0], refVec[i].m_max_corner.m_values[1]);
        p2.dimension = 2;
        refMBRs.push_back(pair<Point, Point>(p1, p2));
    }
    // if there are no first level MBRs
    if(refMBRs.size() == 0){
        refMBRs.push_back(pair<Point, Point>(ref.bound));
    }
    ref.FirstLevelMBRs = refMBRs;
    
    // do randomize for both query and data
    ref.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    priority_queue<double> krecord; // descending
    
    clock_t start, stop;
    start = clock();
    
    // random select k point cloud
    double distance = 0;
    for(int i = 0; i < k; i++){
        distance = ExactHausdorff::PAMI2015(ref, dataset[i]);
        krecord.push(distance);
    }
    
    double kthValue = krecord.top();
    
    // calculate MBRs and prune, using MBRs to prune first
    for(int i = k; i < dataset.size(); i++){
        
        distance = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[i].FirstLevelMBRs); // no need to worry whether there is MBRs or not
        
        if(distance > kthValue){
            continue;
        } else {
            distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
            if(distance < kthValue) {
                krecord.pop();
                krecord.push(distance);
                kthValue = krecord.top();
            }
        }
    }
    
    stop = clock();
    
    cout << "combined method time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

void KNNSearch::KNN_COMBINED_KCenter(PointCloud &ref, int k, int number){
    rtree refRTree(ref.pointcloud.begin(), ref.pointcloud.end());
    
    // calculate ref's bound
    RTV::box_type refbound = refRTree.bounds();
    Point p1(refbound.m_min_corner.m_values[0], refbound.m_min_corner.m_values[1]);
    p1.dimension = 2;
    Point p2(refbound.m_max_corner.m_values[0], refbound.m_max_corner.m_values[1]);
    p2.dimension = 2;
    ref.bound = pair<Point, Point>(p1, p2);
    
    // calculate ref's MBRs
    vector<RTV::box_type> refVec = getMBRs2(refRTree, number); // extract the first level
    vector<pair<Point, Point>> refMBRs;
    for(int i = 0; i < refVec.size(); i++){
        Point p1(refVec[i].m_min_corner.m_values[0], refVec[i].m_min_corner.m_values[1]);
        p1.dimension = 2;
        Point p2(refVec[i].m_max_corner.m_values[0], refVec[i].m_max_corner.m_values[1]);
        p2.dimension = 2;
        refMBRs.push_back(pair<Point, Point>(p1, p2));
    }
    // if there are no first level MBRs
    if(refMBRs.size() == 0){
        refMBRs.push_back(pair<Point, Point>(ref.bound));
    }
    ref.FirstLevelMBRs = refMBRs;
    
    // do KCenter ordering
    ref.sortByKcenter();
//    ref.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    priority_queue<double> krecord; // descending
    
    clock_t start, stop;
    start = clock();
    
    // random select k point cloud
    double distance = 0;
    for(int i = 0; i < k; i++){
        distance = ExactHausdorff::PAMI2015(ref, dataset[i]);
        krecord.push(distance);
    }
    
    double kthValue = krecord.top();
    
    // calculate MBRs and prune, using MBRs to prune first
    for(int i = k; i < dataset.size(); i++){
        
        distance = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[i].FirstLevelMBRs); // no need to worry whether there is MBRs or not
        
        if(distance > kthValue){
            continue;
        } else {
            distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
            if(distance < kthValue) {
                krecord.pop();
                krecord.push(distance);
                kthValue = krecord.top();
            }
        }
    }
    
    stop = clock();
    
    cout << "combined kcenter method time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

void KNNSearch::KNN_COMBINED_KCenter_LB(PointCloud &ref, int k, int number){
    
    ref.generateBoundAndMBRs(10);
    vector<pair<double,int>> disToKcenter = ref.sortByKcenterWithRecord();

    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    priority_queue<double> krecord; // descending
    
    clock_t start, stop;
    start = clock();
    
    // select k point cloud, calculate its exact HD
    double distance = 0;
    for(int i = 0; i < k; i++){
        distance = ExactHausdorff::PAMI2015(ref, dataset[i]);
        krecord.push(distance);
    }
    
    double kthValue = krecord.top();
    double kcenterNum = ref.getKCenterNum();
    double maxLB = 0;
    int kcenterIndex = 0;
    bool breakTag = false;
    int breakcount = 0;
    
    // calculate MBRs and prune, using MBRs to prune first
    for(int i = k; i < dataset.size(); i++){
        // first calculate its MBRs LB (i.e., ENHLB)
        distance = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[i].FirstLevelMBRs); // no need to worry whether there is MBRs or not
        // try to prune by ENHLB
        if(distance > kthValue){
            continue;
        } else {
            ExactHausdorff::maxAndDistances result = ExactHausdorff::Partial_PAMI2015_Pruning_WithRecord_FullInnerloop(ref, dataset[i], 0, kcenterNum, 0, kthValue);
            if(result.max == -1){
                // pruned during the partialHD
                continue;
            }else if(kcenterNum < ref.pointcloud.size()){
                // calculate LB first
                maxLB = 0;
                breakTag = false;
                for(int i = kcenterNum, j=0; i < ref.pointcloud.size(); i++,j++){
                    kcenterIndex = disToKcenter[j].second;
                    // 错的！！！ 单边的hausdorff distance 不满足三角不等式！！！
                    distance = fabs(result.distances[kcenterIndex] - disToKcenter[j].first); // if you use this to prune, make sure the above inner loop calculate the NN
                    if(distance > maxLB){
                        maxLB = distance;
                        if(maxLB > kthValue){
                            breakTag = true;
                            break;
                        }
                    }
                }
                // pruned by KcenterLB
                if(breakTag){
                    breakcount++;
                    continue;
                } else {
                // calculate the remaining HD
                    distance = ExactHausdorff::Partial_PAMI2015_Pruning(ref, dataset[i], kcenterNum, ref.pointcloud.size(), result.max, kthValue);
                    // pruned during calculating remaining HD
                    if(distance == -1){
                        continue;
                    }else if(distance < kthValue) {
                        krecord.pop();
                        krecord.push(distance);
                        kthValue = krecord.top();
                    }
                }
            } else { // finish the loop already
                // update krecord;
                if(result.max < kthValue){
                    krecord.pop();
                    krecord.push(result.max);
                    kthValue = krecord.top();
                }
            }
        }
    }
    
    stop = clock();
    
    cout << "combined kcenter LB method time usage: " << stop-start << endl;
    cout << "break by KCenter LB count: " << breakcount << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

void KNNSearch::KNN_COMBINED_KCenter_UB(PointCloud &ref, int k, int number){
    ref.generateBoundAndMBRs(10);
    vector<pair<double,int>> disToKcenter = ref.sortByKcenterWithRecord();
    
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    priority_queue<double> krecord; // descending
    
    clock_t start, stop;
    start = clock();
    
    // select k point cloud, calculate its exact HD
    double distance = 0;
    for(int i = 0; i < k; i++){
        distance = ExactHausdorff::PAMI2015(ref, dataset[i]);
        krecord.push(distance);
    }
    
    double kthValue = krecord.top();
    double kcenterNum = ref.getKCenterNum();
    double maxLB = 0;
    int kcenterIndex = 0;
    bool breakTag = false;
    
    // calculate MBRs and prune, using MBRs to prune first
    for(int i = k; i < dataset.size(); i++){
        // first calculate its MBRs LB (i.e., ENHLB)
        distance = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[i].FirstLevelMBRs); // no need to worry whether there is MBRs or not
        // try to prune by ENHLB
        if(distance > kthValue){
            continue;
        } else {
            ExactHausdorff::maxAndDistances result = ExactHausdorff::Partial_PAMI2015_Pruning_WithRecord(ref, dataset[i], 0, kcenterNum, 0, kthValue);
            if(result.max == -1){
                // pruned during the partialHD
                continue;
            }else if(kcenterNum < ref.pointcloud.size()){
                distance = ExactHausdorff::Partial_PAMI2015_Pruning_KCenterUB(ref, dataset[i], kcenterNum, ref.pointcloud.size(), result.max, kthValue, disToKcenter, result);
                // pruned during calculating remaining HD
                if(distance == -1){
                    continue;
                }else if(distance < kthValue) {
                    krecord.pop();
                    krecord.push(distance);
                    kthValue = krecord.top();
                }
            } else {
                // finish the loop already
                // update krecord;
                if(result.max < kthValue){
                    krecord.pop();
                    krecord.push(result.max);
                    kthValue = krecord.top();
                }
            }
        }
    }
    
    stop = clock();
    
    cout << "combined kcenter UB method time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

void KNNSearch::KNN_COMBINED_KCenter_UB2(PointCloud &ref, int k, int number, int dataSizeThreshold){
    ref.generateBoundAndMBRs(10);
    vector<pair<double,int>> disToKcenter = ref.sortByKcenterWithRecord();
    
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    priority_queue<double> krecord; // descending
    
    clock_t start, stop;
    start = clock();
    
    // select k point cloud, calculate its exact HD
    double distance = 0;
    for(int i = 0; i < k; i++){
        distance = ExactHausdorff::PAMI2015(ref, dataset[i]);
        krecord.push(distance);
    }
    
    double kthValue = krecord.top();
    
    // calculate MBRs and prune, using MBRs to prune first
    for(int i = k; i < dataset.size(); i++){
        distance = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[i].FirstLevelMBRs); // no need to worry whether there is MBRs or not
        if(distance > kthValue){
            continue;
        } else {
            if(dataset[i].pointcloud.size() < dataSizeThreshold){
                distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
            } else {
                distance = ExactHausdorff::PAMI2015_Pruning_KCenterUB(ref, dataset[i], disToKcenter, kthValue);
            }
            if(distance == -1){
                continue;
            }else if(distance < kthValue){
                krecord.pop();
                krecord.push(distance);
                kthValue = krecord.top();
            }
        }
    }
    
    stop = clock();
    
    cout << "combined kcenter UB 2 method time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

// ==================== my method ==========================

struct prqnode{
    int pcindex; // the data point cloud index in dataset
    int progress; // the query point cloud's current point
    int level; // 0 for partial, 1 for exact;  In MINE2: 0 for basicLB, 1 for ENHLB, 2 for PartialLB, 3 for Exact
    double distance; // the current Hausdorff Distance
    prqnode(int pci, int pro, int lev, double dis){
        pcindex = pci;
        progress = pro;
        level = lev;
        distance = dis;
    }
};

struct cmp_prqnode{
    bool operator()(prqnode &n1, prqnode &n2){
        return n1.distance > n2.distance; // ascending order
    }
};

// consider data's size!!!!
// a : times of K
// b : percentage of Query
// c : threshold for data point cloud
void KNNSearch::KNN_MINE(PointCloud &ref, int k, double a, double b, int c, int querythreshold){
    // 1. randomize query point cloud and dataset as preprocessing
    // 2. calculate a*K exact HD (random selected from data), using the Kth one for pruning (suitable for large dataset and small K, a is a parameter)
    // 3.1 for the rest, calculate partial HD as LB (b * number of points in Query), if LB > Kth value, prune, else, store it in priority queue
    // 3.2 for the above step, if data point cloud is small, calculate the exact HD directly
    // 4. after all are calculated, keep priority queue and pruning to get the KNN result
    
    vector<prqnode> result;
    priority_queue<prqnode, vector<prqnode>, cmp_prqnode> prqueue; // ascending
    priority_queue<double> krecord; // descending
    
    ref.randomize();
    randomizeDataset();
    
    long querySize = ref.pointcloud.size();
    long datasetSize = dataset.size();
    long partialQuerySize = b*querySize;
    partialQuerySize = partialQuerySize < querythreshold? partialQuerySize : querythreshold;
    int ak = a*k; // a times k
    
    clock_t start, stop;
    start = clock();
    
    // calculate a*k exact HD
    for(int i = 0; i < ak; i++){
        double distance = ExactHausdorff::PAMI2015(ref, dataset[i]);
        prqnode node(i, querySize, 1, distance);
        prqueue.push(node);
        
        if(krecord.size() < k){
            krecord.push(distance);
        } else { // =k
            double kth = krecord.top();
            if(distance < kth){
                krecord.pop();
                krecord.push(distance);
            } else {
                // do noting
            }
        }
    }

    double kthValue = krecord.top();
    double max = 0;
    double min = std::numeric_limits<double>::infinity(); // infinity
    double distance = 0;
    
    // calculate partial HD for the rest
    for(int i = ak; i < datasetSize; i++){
        max = 0;
        distance = 0;
        if(dataset[i].pointcloud.size() > c){ // calculate partial HD
            for (int i1 = 0; i1 < partialQuerySize; i1++){
                min = std::numeric_limits<double>::infinity();
                for(int j = 0; j < dataset[i].pointcloud.size(); j++){
                    distance = ref.pointcloud[i1].distanceTo(dataset[i].pointcloud[j]);
                    if(distance < min){
                        min = distance;
                    }
                    if(distance <= max){
                        break;
                    }
                }
                if(min > max){
                    max = min;
                    if(max >= kthValue){
                        break; // give up this point cloud
                    }
                }
                
            }
            prqnode node(i, partialQuerySize, 0, max); // the partial Hausdorff distance
            prqueue.push(node);
        } else { // calculate full HD
            for (int i1 = 0; i1 < querySize; i1++){
                min = std::numeric_limits<double>::infinity();
                for(int j = 0; j < dataset[i].pointcloud.size(); j++){
                    distance = ref.pointcloud[i1].distanceTo(dataset[i].pointcloud[j]);
                    if(distance < min){
                        min = distance;
                    }
                    if(distance <= max){
                        break;
                    }
                }
                if(min > max){
                    max = min;
                    if(max >= kthValue){
                        break; // give up this point cloud
                    }
                }
                
            }
            prqnode node(i, querySize, 1, max); // the partial Hausdorff distance
            prqueue.push(node);
            if(node.distance < kthValue){
                krecord.pop();
                krecord.push(distance);
                kthValue = krecord.top();
            }
        }
    }
    
    int tempk = k;
    while(tempk){
        prqnode node = prqueue.top();
        prqueue.pop();
        if(node.level == 1){
            tempk--;
            if(node.distance < kthValue){
                krecord.pop();
                krecord.push(distance);
                kthValue = krecord.top();
            }
            result.push_back(node);
        } else {
            // keep calculating the reamining part
            int i = node.pcindex;
            max = node.distance;
            distance = node.distance;
            for (int i1 = partialQuerySize; i1 < querySize; i1++){
                min = std::numeric_limits<double>::infinity();
                for(int j = 0; j < dataset[i].pointcloud.size(); j++){
                    distance = ref.pointcloud[i1].distanceTo(dataset[i].pointcloud[j]);
                    if(distance < min){
                        min = distance;
                    }
                    if(distance <= max){
                        break;
                    }
                }
                if(min > max){
                    max = min;
                    if(max >= kthValue){
                        break; // give up this point cloud
                    }
                }
            }
            node.progress = querySize;
            node.level = 1;
            node.distance = max;
            prqueue.push(node);
        }
    }
    
    stop = clock();
    
    cout << "my method time usage: " << stop - start << endl;
    for(int i = 0; i < k; i++){
        cout << "i: " << result[i].distance << " " << result[i].pcindex << endl;
    }
    
}

void KNNSearch::KNN_MINE2(PointCloud &ref, int k, int partialQuerySize){
    
    vector<prqnode> result;
    priority_queue<prqnode, vector<prqnode>, cmp_prqnode> prqueue;
    
    rtree refRTree(ref.pointcloud.begin(), ref.pointcloud.end());
    
    long refsize = ref.pointcloud.size();
    partialQuerySize = partialQuerySize < refsize ? partialQuerySize : refsize;
    
    // calculate ref's bound
    RTV::box_type refbound = refRTree.bounds();
    Point p1(refbound.m_min_corner.m_values[0], refbound.m_min_corner.m_values[1]);
    p1.dimension = 2;
    Point p2(refbound.m_max_corner.m_values[0], refbound.m_max_corner.m_values[1]);
    p2.dimension = 2;
    ref.bound = pair<Point, Point>(p1, p2);
    
    // calculate ref's MBRs
    vector<RTV::box_type> refVec = getMBRs(refRTree); // extract the first level
    vector<pair<Point, Point>> refMBRs;
    for(int i = 0; i < refVec.size(); i++){
        Point p1(refVec[i].m_min_corner.m_values[0], refVec[i].m_min_corner.m_values[1]);
        p1.dimension = 2;
        Point p2(refVec[i].m_max_corner.m_values[0], refVec[i].m_max_corner.m_values[1]);
        p2.dimension = 2;
        refMBRs.push_back(pair<Point, Point>(p1, p2));
    }
    if(refMBRs.size() == 0){
        refMBRs.push_back(pair<Point, Point>(ref.bound));
    }
    ref.FirstLevelMBRs = refMBRs;
    
    // randomize
    ref.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    clock_t start,stop;
    start = clock();
    
    double distance = 0;
    for(int i = 0; i < dataset.size(); i++){
        distance = HausdorffDistanceForBound(ref.bound, dataset[i].bound);
        prqnode node(i, 0, 0, distance);
        prqueue.push(node);
    }
    
    int _k = k;
    
    int count1 = 0, count2 = 0, count3 = 0, nbcount = 0, nombrcount = 0;
    double max = 0, min = 0;
    
    while(k){
        prqnode node = prqueue.top();
        prqueue.pop();
        if(node.level == 0){
            // consider if there is not multi level
            if(dataset[node.pcindex].FirstLevelMBRs.size() == 1 && ref.FirstLevelMBRs.size() == 1){
                // should we calculate partial HD or exact HD instead here?
                max = 0;
                for (int i = 0; i < partialQuerySize; i++){
                    min = std::numeric_limits<double>::infinity();
                    for(int j = 0; j < dataset[node.pcindex].pointcloud.size(); j++){
                        distance = ref.pointcloud[i].distanceTo(dataset[node.pcindex].pointcloud[j]);
                        if(distance < min){
                            min = distance;
                        }
                        if(distance <= max){
                            break;
                        }
                    }
                    if(min > max){
                        max = min;
                        //                    if(max >= kthValue){
                        //                        break; // give up this point cloud
                        //                    }
                    }
                }
                node.progress = partialQuerySize;
                node.level = 2;
                count2++;
                nombrcount++;
                node.distance = max;
            } else {
                // calculate ENHLB
                node.distance = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[node.pcindex].FirstLevelMBRs); // this refVec is not enhanced !!!!!!!!!!
                count1++;
            }
            node.level = 1;
            prqueue.push(node);
        } else if(node.level == 1){
            // calculate partial HD as LB
            max = 0;
            for (int i = 0; i < partialQuerySize; i++){
                min = std::numeric_limits<double>::infinity();
                for(int j = 0; j < dataset[node.pcindex].pointcloud.size(); j++){
                    distance = ref.pointcloud[i].distanceTo(dataset[node.pcindex].pointcloud[j]);
                    if(distance < min){
                        min = distance;
                    }
                    if(distance <= max){
                        break;
                    }
                }
                if(min > max){
                    max = min;
//                    if(max >= kthValue){
//                        break; // give up this point cloud
//                    }
                }
            }
            node.progress = partialQuerySize;
            node.level = 2;
            if(max > node.distance){
                nbcount++; // nearly all of them is bigger
            } else {
                // ???? will this be even bigger?
            }
            node.distance = max;
            prqueue.push(node);
            count2++;
        } else if(node.level == 2){
            // calculate the reamining HD
            max = node.distance;
            for (int i = partialQuerySize; i < refsize; i++){
                min = std::numeric_limits<double>::infinity();
                for(int j = 0; j < dataset[node.pcindex].pointcloud.size(); j++){
                    distance = ref.pointcloud[i].distanceTo(dataset[node.pcindex].pointcloud[j]);
                    if(distance < min){
                        min = distance;
                    }
                    if(distance <= max){
                        break;
                    }
                }
                if(min > max){
                    max = min;
//                    if(max >= kthValue){
//                        break; // give up this point cloud
//                    }
                }
            }
            node.progress = refsize;
            node.level = 3;
            node.distance = max;
            prqueue.push(node);
            count3++;
        } else if(node.level == 3){
            result.push_back(node);
            k--;
        }
    }
    stop = clock();
    cout << "MINE_2 time usage: " <<  stop - start << endl;
    cout << "filtering count: count1=" << count1 << "  count2=" << count2 << "  count3=" << count3 << "  nbcount=" << nbcount << "  nombrcount=" << nombrcount << endl;
    for(int i = 0; i < _k; i++){
        cout << result[i].distance << endl;
    }
}

// ignore BscLB, calculate partial HD according to percentage.
// 不能直接算ENHLB 因为有些没有啊！
void KNNSearch::KNN_MINE3(PointCloud &ref, int k, double percentage, int LB, int UB, int queryMBRNumber){
    
    vector<prqnode> result;
    priority_queue<prqnode, vector<prqnode>, cmp_prqnode> prqueue;
    long refsize = ref.pointcloud.size();
    int partialQuerySize = refsize*percentage;
    if(partialQuerySize > UB){
        partialQuerySize = UB;
    }
    if(partialQuerySize < LB){
        partialQuerySize = LB;
    }
    if(partialQuerySize > ref.pointcloud.size()){
        partialQuerySize = ref.pointcloud.size();
    }

    rtree refRTree(ref.pointcloud.begin(), ref.pointcloud.end());
    // calculate ref's bound
    RTV::box_type refbound = refRTree.bounds();
    Point p1(refbound.m_min_corner.m_values[0], refbound.m_min_corner.m_values[1]);
    p1.dimension = 2;
    Point p2(refbound.m_max_corner.m_values[0], refbound.m_max_corner.m_values[1]);
    p2.dimension = 2;
    ref.bound = pair<Point, Point>(p1, p2);
    
    // calculate ref's MBRs
    vector<RTV::box_type> refVec = getMBRs2(refRTree, queryMBRNumber); // extract the first level
    vector<pair<Point, Point>> refMBRs;
    for(int i = 0; i < refVec.size(); i++){
        Point p1(refVec[i].m_min_corner.m_values[0], refVec[i].m_min_corner.m_values[1]);
        p1.dimension = 2;
        Point p2(refVec[i].m_max_corner.m_values[0], refVec[i].m_max_corner.m_values[1]);
        p2.dimension = 2;
        refMBRs.push_back(pair<Point, Point>(p1, p2));
    }
    // if there are no first level MBRs
    if(refMBRs.size() == 0){
        refMBRs.push_back(pair<Point, Point>(ref.bound));
    }
    ref.FirstLevelMBRs = refMBRs;
    
    // do randomize for both query and data
    ref.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    clock_t start,stop;
    start = clock();
    
//    clock_t start1, stop1; // you can remove this later
    
//    start1 = clock(); // you can remove this later
    double distance = 0;
    for(int i = 0; i < dataset.size(); i++){
        distance = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[i].FirstLevelMBRs);
        prqnode node(i, 0, 0, distance);
        prqueue.push(node);
    }
//    stop1 = clock(); // you can remove this later
    
    int _k = k;
    
    int count1 = 0, count2 = 0;
    double max = 0, min = 0;
    
//    clock_t start2, stop2, start3, stop3, totalPHD = 0, totalHD = 0;
    
//    ofstream outfile1, outfile2; // you can remove this later
//    outfile1.open("/Users/lizhe/Desktop/reports/KNN8/Testing3-prqueue-pop"); // you can remove this later
//    outfile2.open("/Users/lizhe/Desktop/reports/KNN8/Testing3-prqueue-kth"); // you can remove this later
//    priority_queue<double> krecord; // descending, you can remove this later
//    int whilecount = 0; // you can remove this later
//    double tempdistance = 0; // you can remove this later
    
    while(k){
//        whilecount++; // you can remove this later
        prqnode node = prqueue.top();
        prqueue.pop();
//        tempdistance = node.distance; // you can remove this later
        if(node.level == 0){
//            start2 = clock(); // you can remove this later
//             calculate partial HD as LB
            max = 0;
            for (int i = 0; i < partialQuerySize; i++){
                min = std::numeric_limits<double>::infinity();
                for(int j = 0; j < dataset[node.pcindex].pointcloud.size(); j++){
                    distance = ref.pointcloud[i].distanceTo(dataset[node.pcindex].pointcloud[j]);
                    if(distance < min){
                        min = distance;
                    }
                    if(distance <= max){
                        break;
                    }
                }
                if(min > max){
                    max = min;
                }
            }
            node.progress = partialQuerySize;
            node.level = 1;
            node.distance = max;
            prqueue.push(node);
            count1++;
//            stop2 = clock(); // you can remove this later
//            totalPHD += stop2-start2; // you can remove this later
        } else if(node.level == 1){
//            start3 = clock(); // you can remove this later
            // calculate the reamining HD
            max = node.distance;
            for (int i = partialQuerySize; i < refsize; i++){
                min = std::numeric_limits<double>::infinity();
                for(int j = 0; j < dataset[node.pcindex].pointcloud.size(); j++){
                    distance = ref.pointcloud[i].distanceTo(dataset[node.pcindex].pointcloud[j]);
                    if(distance < min){
                        min = distance;
                    }
                    if(distance <= max){
                        break;
                    }
                }
                if(min > max){
                    max = min;
                }
            }
            node.progress = refsize;
            node.level = 2;
            node.distance = max;
            prqueue.push(node);
            count2++;
//            stop3 = clock(); // you can remove this later
//            totalHD += stop3 - start3; // you can remove this later
           
            // you can remove this later
//            if(krecord.size() <_k){
//                krecord.push(max);
//            } else if(max < krecord.top()){
//                krecord.pop();
//                krecord.push(max);
//            }
        } else if(node.level == 2){
            result.push_back(node);
            k--;
        }
        // you can remove this later
//        if(krecord.size() < _k){
//            outfile1 << whilecount << " " << tempdistance << endl;
//        } else{
//            outfile1 << whilecount << " " << tempdistance << endl;
//            outfile2 << whilecount << " " << krecord.top() << endl;
//        }
    }
    stop = clock();
    cout << "MINE_3 time usage: " <<  stop - start << endl;
    cout << "filtering count: count1=" << count1 << "  count2=" << count2 << endl;
//    cout << "ENHLB time: " << stop1-start1 << "  partial HD time: " << totalPHD << "  exact HD time" << totalHD << endl; // you can remove this later
    for(int i = 0; i < _k; i++){
        cout << result[i].distance << endl;
    }
}


void KNNSearch::KNN_MINE4(PointCloud &ref, int k, double percentage, int LB, int UB, int queryMBRNumber){
    
    vector<prqnode> result;
    priority_queue<prqnode, vector<prqnode>, cmp_prqnode> prqueue;
    long refsize = ref.pointcloud.size();
    int partialQuerySize = refsize*percentage;
    if(partialQuerySize > UB){
        partialQuerySize = UB;
    }
    if(partialQuerySize < LB){
        partialQuerySize = LB;
    }
    if(partialQuerySize > ref.pointcloud.size()){
        partialQuerySize = ref.pointcloud.size();
    }
    
    rtree refRTree(ref.pointcloud.begin(), ref.pointcloud.end());
    // calculate ref's bound
    RTV::box_type refbound = refRTree.bounds();
    Point p1(refbound.m_min_corner.m_values[0], refbound.m_min_corner.m_values[1]);
    p1.dimension = 2;
    Point p2(refbound.m_max_corner.m_values[0], refbound.m_max_corner.m_values[1]);
    p2.dimension = 2;
    ref.bound = pair<Point, Point>(p1, p2);
    
    // calculate ref's MBRs
    vector<RTV::box_type> refVec = getMBRs2(refRTree, queryMBRNumber); // extract the first level
    vector<pair<Point, Point>> refMBRs;
    for(int i = 0; i < refVec.size(); i++){
        Point p1(refVec[i].m_min_corner.m_values[0], refVec[i].m_min_corner.m_values[1]);
        p1.dimension = 2;
        Point p2(refVec[i].m_max_corner.m_values[0], refVec[i].m_max_corner.m_values[1]);
        p2.dimension = 2;
        refMBRs.push_back(pair<Point, Point>(p1, p2));
    }
    // if there are no first level MBRs
    if(refMBRs.size() == 0){
        refMBRs.push_back(pair<Point, Point>(ref.bound));
    }
    ref.FirstLevelMBRs = refMBRs;
    
    // do kcenter ordering !!!!!!!!!!!
    ref.sortByKcenter();
    // assume the dataset is already kcenter minzed !!  But actually we don not need it be ordered right?
//    for(int i = 0; i < dataset.size(); i++){
//        dataset[i].sortByKcenter();
//    }
    
    clock_t start,stop;
    start = clock();
    
//    clock_t start1, stop1; // you can remove this later
    
//    start1 = clock(); // you can remove this later
    double distance = 0;
    for(int i = 0; i < dataset.size(); i++){
        distance = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[i].FirstLevelMBRs);
        prqnode node(i, 0, 0, distance);
        prqueue.push(node);
    }
//    stop1 = clock(); // you can remove this later
    
    int _k = k;
    
    int count1 = 0, count2 = 0;
    double max = 0, min = 0;
    
//    clock_t start2, stop2, start3, stop3, totalPHD = 0, totalHD = 0; // you can remove this later
    
//    ofstream outfile1, outfile2; // you can remove this later
//    outfile1.open("/Users/lizhe/Desktop/reports/KNN8/Testing3-prqueue-pop"); // you can remove this later
//    outfile2.open("/Users/lizhe/Desktop/reports/KNN8/Testing3-prqueue-kth"); // you can remove this later
//    priority_queue<double> krecord; // descending, you can remove this later
//    int whilecount = 0; // you can remove this later
//    double tempdistance = 0; // you can remove this later
    
    while(k){
//        whilecount++; // you can remove this later
        prqnode node = prqueue.top();
        prqueue.pop();
//        tempdistance = node.distance; // you can remove this later
        if(node.level == 0){
//            start2 = clock(); // you can remove this later
            //             calculate partial HD as LB
            max = 0;
            for (int i = 0; i < partialQuerySize; i++){
                min = std::numeric_limits<double>::infinity();
                for(int j = 0; j < dataset[node.pcindex].pointcloud.size(); j++){
                    distance = ref.pointcloud[i].distanceTo(dataset[node.pcindex].pointcloud[j]);
                    if(distance < min){
                        min = distance;
                    }
                    if(distance <= max){
                        break;
                    }
                }
                if(min > max){
                    max = min;
                }
            }
            node.progress = partialQuerySize;
            node.level = 1;
            node.distance = max;
            prqueue.push(node);
            count1++;
//            stop2 = clock(); // you can remove this later
//            totalPHD += stop2-start2; // you can remove this later
        } else if(node.level == 1){
//            start3 = clock(); // you can remove this later
            // calculate the reamining HD
            max = node.distance;
            for (int i = partialQuerySize; i < refsize; i++){
                min = std::numeric_limits<double>::infinity();
                for(int j = 0; j < dataset[node.pcindex].pointcloud.size(); j++){
                    distance = ref.pointcloud[i].distanceTo(dataset[node.pcindex].pointcloud[j]);
                    if(distance < min){
                        min = distance;
                    }
                    if(distance <= max){
                        break;
                    }
                }
                if(min > max){
                    max = min;
                }
            }
            node.progress = refsize;
            node.level = 2;
            node.distance = max;
            prqueue.push(node);
            count2++;
//            stop3 = clock(); // you can remove this later
//            totalHD += stop3 - start3; // you can remove this later
            
            // you can remove this later
//            if(krecord.size() <_k){
//                krecord.push(max);
//            } else if(max < krecord.top()){
//                krecord.pop();
//                krecord.push(max);
//            }
        } else if(node.level == 2){
            result.push_back(node);
            k--;
        }
        // you can remove this later
//        if(krecord.size() < _k){
//            outfile1 << whilecount << " " << tempdistance << endl;
//        } else{
//            outfile1 << whilecount << " " << tempdistance << endl;
//            outfile2 << whilecount << " " << krecord.top() << endl;
//        }
    }
    stop = clock();
    cout << "MINE_4 time usage: " <<  stop - start << endl;
    cout << "filtering count: count1=" << count1 << "  count2=" << count2 << endl;
//    cout << "ENHLB time: " << stop1-start1 << "  partial HD time: " << totalPHD << "  exact HD time" << totalHD << endl; // you can remove this later
    for(int i = 0; i < _k; i++){
        cout << result[i].distance << endl;
    }
}

// using kcenter ordering, using threshold in PartialHD
void KNNSearch::KNN_MINE5(PointCloud &ref, int k, double percentage, int LB, int UB, int queryMBRNumber, double KNNValue){
    
    vector<prqnode> result;
    priority_queue<prqnode, vector<prqnode>, cmp_prqnode> prqueue;
    long refsize = ref.pointcloud.size();
    int partialQuerySize = refsize*percentage;
    if(partialQuerySize > UB){
        partialQuerySize = UB;
    }
    if(partialQuerySize < LB){
        partialQuerySize = LB;
    }
    if(partialQuerySize > ref.pointcloud.size()){
        partialQuerySize = ref.pointcloud.size();
    }
    
    ref.generateBoundAndMBRs(queryMBRNumber);
    ref.sortByKcenter();
    
    clock_t start,stop;
    start = clock();
    
    //    clock_t start1, stop1; // you can remove this later
    //    start1 = clock(); // you can remove this later
    double distance = 0;
    for(int i = 0; i < dataset.size(); i++){
        distance = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[i].FirstLevelMBRs);
        prqnode node(i, 0, 0, distance);
        prqueue.push(node);
    }
    //    stop1 = clock(); // you can remove this later
    
    int _k = k;
    
    int count1 = 0, count2 = 0, countbreak1 = 0, countbreak2 = 0;
    double max = 0, min = 0;
    
    //    clock_t start2, stop2, start3, stop3, totalPHD = 0, totalHD = 0; // you can remove this later
    
    //    ofstream outfile1, outfile2; // you can remove this later
    //    outfile1.open("/Users/lizhe/Desktop/reports/KNN8/Testing3-prqueue-pop"); // you can remove this later
    //    outfile2.open("/Users/lizhe/Desktop/reports/KNN8/Testing3-prqueue-kth"); // you can remove this later
    //    priority_queue<double> krecord; // descending, you can remove this later
    //    int whilecount = 0; // you can remove this later
    //    double tempdistance = 0; // you can remove this later
    
    while(k){
        //        whilecount++; // you can remove this later
        prqnode node = prqueue.top();
        prqueue.pop();
        //        tempdistance = node.distance; // you can remove this later
        if(node.level == 0){
            //            start2 = clock(); // you can remove this later
            //             calculate partial HD as LB
            max = 0;
            max = ExactHausdorff::Partial_PAMI2015_Pruning(ref, dataset[node.pcindex], 0, partialQuerySize, 0, KNNValue);
            //            for (int i = 0; i < partialQuerySize; i++){
            //                min = std::numeric_limits<double>::infinity();
            //                for(int j = 0; j < dataset[node.pcindex].pointcloud.size(); j++){
            //                    distance = ref.pointcloud[i].distanceTo(dataset[node.pcindex].pointcloud[j]);
            //                    if(distance < min){
            //                        min = distance;
            //                    }
            //                    if(distance <= max){
            //                        break;
            //                    }
            //                }
            //                if(min > max){
            //                    max = min;
            //                }
            //            }
            count1++;
            if(max != -1){
                node.progress = partialQuerySize;
                node.level = 1;
                node.distance = max;
                prqueue.push(node);
            } else{
                countbreak1++;
                continue;
            }
            //            stop2 = clock(); // you can remove this later
            //            totalPHD += stop2-start2; // you can remove this later
        } else if(node.level == 1){
            //            start3 = clock(); // you can remove this later
            // calculate the reamining HD
            max = node.distance;
            max = ExactHausdorff::Partial_PAMI2015_Pruning(ref, dataset[node.pcindex], partialQuerySize, refsize, max, KNNValue);
            //            for (int i = partialQuerySize; i < refsize; i++){
            //                min = std::numeric_limits<double>::infinity();
            //                for(int j = 0; j < dataset[node.pcindex].pointcloud.size(); j++){
            //                    distance = ref.pointcloud[i].distanceTo(dataset[node.pcindex].pointcloud[j]);
            //                    if(distance < min){
            //                        min = distance;
            //                    }
            //                    if(distance <= max){
            //                        break;
            //                    }
            //                }
            //                if(min > max){
            //                    max = min;
            //                }
            //            }
            count2++;
            if(max != -1){
                node.progress = refsize;
                node.level = 2;
                node.distance = max;
                prqueue.push(node);
            } else{
                countbreak2++;
                continue;
            }
            
            //            stop3 = clock(); // you can remove this later
            //            totalHD += stop3 - start3; // you can remove this later
            
            // you can remove this later
            //            if(krecord.size() <_k){
            //                krecord.push(max);
            //            } else if(max < krecord.top()){
            //                krecord.pop();
            //                krecord.push(max);
            //            }
        } else if(node.level == 2){
            result.push_back(node);
            k--;
        }
        // you can remove this later
        //        if(krecord.size() < _k){
        //            outfile1 << whilecount << " " << tempdistance << endl;
        //        } else{
        //            outfile1 << whilecount << " " << tempdistance << endl;
        //            outfile2 << whilecount << " " << krecord.top() << endl;
        //        }
    }
    stop = clock();
    cout << "MINE_5 time usage: " <<  stop - start << endl;
    cout << "filtering count: count1=" << count1 << "  count2=" << count2 << "  countbreak1=" << countbreak1 << "  countbreak2=" << countbreak2 << endl;
    //    cout << "ENHLB time: " << stop1-start1 << "  partial HD time: " << totalPHD << "  exact HD time" << totalHD << endl; // you can remove this later
    for(int i = 0; i < _k; i++){
        cout << result[i].distance << endl;
    }
}

// using kcenter ordering, and gradually improve progress
// step should less than query's size!
void KNNSearch::KNN_MINE6(PointCloud &ref, int k, int step, int queryMBRNumber, double percent, int lowerthreshold, int upperthreshold){
    vector<prqnode> result;
    priority_queue<prqnode, vector<prqnode>, cmp_prqnode> prqueue;
    long refsize = ref.pointcloud.size();
    
    ref.generateBoundAndMBRs(queryMBRNumber);
    ref.sortByKcenter(percent, lowerthreshold, upperthreshold);
    
    clock_t start,stop;
    start = clock();

    double distance = 0;
    for(int i = 0; i < dataset.size(); i++){
        distance = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[i].FirstLevelMBRs);
        prqnode node(i, 0, 0, distance);
        prqueue.push(node);
    }
    
    int _k = k;
    
    int count1 = 0;
    double max = 0, min = 0;
    
    while(k){
        prqnode node = prqueue.top();
        prqueue.pop();
        if(node.level == 0){
            max = 0;
            max = ExactHausdorff::Partial_PAMI2015(ref, dataset[node.pcindex], 0, step, 0);
            node.progress = step;
            node.level = 1;
            if(step >= refsize)
                node.level = 2;
            node.distance = max;
            prqueue.push(node);
            count1++;
        } else if(node.level == 1){
            max = ExactHausdorff::Partial_PAMI2015(ref, dataset[node.pcindex], node.progress, node.progress+step, node.distance);
            node.progress += step;
            node.level = 1;
            if(node.progress+step >= refsize)
                node.level = 2;
            node.distance = max;
            prqueue.push(node);
            count1++;
        } else if(node.level == 2){
            result.push_back(node);
            k--;
        }
    }
    stop = clock();
    cout << "MINE_6 time usage: " <<  stop - start << endl;
    cout << "filtering count: count1=" << count1 << endl;
    //    cout << "ENHLB time: " << stop1-start1 << "  partial HD time: " << totalPHD << "  exact HD time" << totalHD << endl; // you can remove this later
    for(int i = 0; i < _k; i++){
        cout << result[i].distance << endl;
    }
}

// size threshold is the threshold for divide the usage of the two methods
void KNNSearch::KNN_MINE7(PointCloud &ref, int k, int step, int queryMBRNumber, double percent, int lowerthreshold, int upperthreshold, int sizeThreshold){
//    vector<prqnode> result;
    priority_queue<double> krecord; // descending
    double kthValue = std::numeric_limits<double>::infinity();
    
    priority_queue<prqnode, vector<prqnode>, cmp_prqnode> prqueue;
    long refsize = ref.pointcloud.size();
    
    ref.generateBoundAndMBRs(queryMBRNumber);
    ref.sortByKcenter(percent, lowerthreshold, upperthreshold);
    
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    // sort dataset according to ascending order.
    sort(dataset.begin(), dataset.end(), cmp_pointcloudsize);
    
    clock_t start,stop;
    start = clock();
    long breakIndex;
    double distance;
    
    for(int i = 0; i < dataset.size(); i++){
        
        if(dataset[i].pointcloud.size() > sizeThreshold){
            breakIndex = i;
            break;
        }
        
        distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        if(distance == -1){
            continue;
        } else if(krecord.size() < k){
            krecord.push(distance);
        } else if(distance < kthValue){
            krecord.pop();
            krecord.push(distance);
            kthValue = krecord.top();
        }
    }
    
    distance = 0;
    for(int i = breakIndex; i < dataset.size(); i++){
        distance = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[i].FirstLevelMBRs);
        if(distance > kthValue){
            continue;
        }
        prqnode node(i, 0, 0, distance);
        prqueue.push(node);
    }
    
    int _k = k;
    
    int count1 = 0, count2 = 0;
    double max = 0, min = 0;
    
    while(k){
        prqnode node = prqueue.top();
        prqueue.pop();
        if(node.distance >= kthValue){
            break;
        }
        
        if(node.level == 0){
            max = 0;
            max = ExactHausdorff::Partial_PAMI2015_Pruning(ref, dataset[node.pcindex], 0, step, 0, kthValue);// should use kthvalue to break during calculation
            if(max == -1){
                continue;
            }
            node.progress = step;
            node.level = 1;
            if(step >= refsize)
                node.level = 2;
            node.distance = max;
            prqueue.push(node);
            count1++;
        } else if(node.level == 1){
            max = ExactHausdorff::Partial_PAMI2015_Pruning(ref, dataset[node.pcindex], node.progress, node.progress+step, node.distance, kthValue);
            if(max == -1){
                continue;
            }
            node.progress += step;
            node.level = 1;
            if(node.progress+step >= refsize)
                node.level = 2;
            node.distance = max;
            prqueue.push(node);
            count1++;
        } else if(node.level == 2){
            count2++;
            if(node.distance < kthValue){
                krecord.pop();
                krecord.push(node.distance);
                kthValue = krecord.top();
            } else{
                continue;
            }
//            result.push_back(node);
//            k--;
        }
    }
    stop = clock();
    cout << "MINE_7 time usage: " <<  stop - start << endl;
    cout << "filtering count: count1=" << count1 << "  count2=" << count2 << endl;
    //    cout << "ENHLB time: " << stop1-start1 << "  partial HD time: " << totalPHD << "  exact HD time" << totalHD << endl; // you can remove this later
//    for(int i = 0; i < _k; i++){
//        cout << result[i].distance << endl;
//    }
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

void KNNSearch::KNN_MINE7_2(PointCloud &ref, int k, int step, int queryMBRNumber, double percent, int lowerthreshold, int upperthreshold, int sizeThreshold){
    priority_queue<double> krecord; // descending
    
    double kthValue = std::numeric_limits<double>::infinity();
    
    priority_queue<prqnode, vector<prqnode>, cmp_prqnode> prqueue;
    long refsize = ref.pointcloud.size();
    
    ref.generateBoundAndMBRs(queryMBRNumber);
    ref.sortByKcenter(percent, lowerthreshold, upperthreshold);
    
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    // sort dataset according to ascending order.
    sort(dataset.begin(), dataset.end(), cmp_pointcloudsize);
    
    clock_t start,stop;
    start = clock();
    long breakIndex;
    double distance;
    
    for(int i = 0; i < dataset.size(); i++){
        
        if(dataset[i].pointcloud.size() > sizeThreshold){
            breakIndex = i;
            break;
        }
        
        distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        if(distance == -1){
            continue;
        } else if(krecord.size() < k){
            krecord.push(distance);
        } else if(distance < kthValue){
            krecord.pop();
            krecord.push(distance);
            kthValue = krecord.top();
        }
    }
    
    distance = 0;
    for(int i = breakIndex; i < dataset.size(); i++){
//        distance = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[i].FirstLevelMBRs);
        distance = ExactHausdorff::Partial_PAMI2015_Pruning(ref, dataset[i], 0, step, 0, kthValue);
        if(distance > kthValue){
            continue;
        }
        prqnode node(i, step, 1, distance);
        prqueue.push(node);
    }
    
    int _k = k;
    
    int count1 = 0, count2 = 0;
    double max = 0, min = 0;
    
    while(k){
        prqnode node = prqueue.top();
        prqueue.pop();
        if(node.distance >= kthValue){
            break;
        }
        
//        if(node.level == 0){
//            max = 0;
//            max = ExactHausdorff::Partial_PAMI2015_Pruning(ref, dataset[node.pcindex], 0, step, 0, kthValue);// should use kthvalue to break during calculation
//            if(max == -1){
//                continue;
//            }
//            node.progress = step;
//            node.level = 1;
//            if(step >= refsize)
//                node.level = 2;
//            node.distance = max;
//            prqueue.push(node);
//            count1++;
        if(node.level == 1){
            max = ExactHausdorff::Partial_PAMI2015_Pruning(ref, dataset[node.pcindex], node.progress, node.progress+step, node.distance, kthValue);
            if(max == -1){
                continue;
            }
            node.progress += step;
            node.level = 1;
            if(node.progress+step >= refsize)
                node.level = 2;
            node.distance = max;
            prqueue.push(node);
            count1++;
        } else if(node.level == 2){
            count2++;
            k--;
            if(node.distance < kthValue){
                krecord.pop();
                krecord.push(node.distance);
                kthValue = krecord.top();
            } else{
                continue;
            }
        }
    }
    stop = clock();
    cout << "MINE_7_2 time usage: " <<  stop - start << endl;
    cout << "filtering count: count1=" << count1 << "  count2=" << count2 << endl;
    //    cout << "ENHLB time: " << stop1-start1 << "  partial HD time: " << totalPHD << "  exact HD time" << totalHD << endl; // you can remove this later
    k = _k;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

void KNNSearch::KNN_MINE7_3(PointCloud &ref, int k, int step, int queryMBRNumber, double percent, int lowerthreshold, int upperthreshold, int sizeThreshold){
    priority_queue<double> krecord; // descending
    
    double kthValue = std::numeric_limits<double>::infinity();
    
    priority_queue<prqnode, vector<prqnode>, cmp_prqnode> prqueue;
    long refsize = ref.pointcloud.size();
    
    ref.generateBoundAndMBRs(queryMBRNumber);
    ref.sortByKcenter(percent, lowerthreshold, upperthreshold);
    
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    // sort dataset according to ascending order.
    sort(dataset.begin(), dataset.end(), cmp_pointcloudsize);
    
    clock_t start,stop;
    start = clock();
    long breakIndex;
    double distance;
    
    for(int i = 0; i < dataset.size(); i++){
        
        if(dataset[i].pointcloud.size() > sizeThreshold){
            breakIndex = i;
            break;
        }
        
        distance = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        if(distance == -1){
            continue;
        } else if(krecord.size() < k){
            krecord.push(distance);
        } else if(distance < kthValue){
            krecord.pop();
            krecord.push(distance);
            kthValue = krecord.top();
        }
    }
    
    distance = 0;
    for(int i = breakIndex; i < dataset.size(); i++){
        distance = HausdorffDistanceForBound(ref.bound, dataset[i].bound);
        if(distance == 0){
            HausdorffDistanceForBound(ref.bound, dataset[i].bound);
            if(distance == -1){
                continue;
            } else if(krecord.size() < k){
                krecord.push(distance);
            } else if(distance < kthValue){
                krecord.pop();
                krecord.push(distance);
                kthValue = krecord.top();
            }
            continue;
        }
        // if not = 0, do the queue method
        distance = ExactHausdorff::Partial_PAMI2015_Pruning(ref, dataset[i], 0, step, 0, kthValue);
        if(distance > kthValue){
            continue;
        }
        prqnode node(i, step, 1, distance);
        prqueue.push(node);
    }
    
    int _k = k;
    
    int count1 = 0, count2 = 0;
    double max = 0, min = 0;
    
    while(k && prqueue.size()){
        prqnode node = prqueue.top();
        prqueue.pop();
        if(node.distance >= kthValue){
            break;
        }
        
        if(node.level == 1){
            max = ExactHausdorff::Partial_PAMI2015_Pruning(ref, dataset[node.pcindex], node.progress, node.progress+step, node.distance, kthValue);
            if(max == -1){
                continue;
            }
            node.progress += step;
            node.level = 1;
            if(node.progress+step >= refsize)
                node.level = 2;
            node.distance = max;
            prqueue.push(node);
            count1++;
        } else if(node.level == 2){
            count2++;
            k--;
            if(node.distance < kthValue){
                krecord.pop();
                krecord.push(node.distance);
                kthValue = krecord.top();
            } else{
                continue;
            }
        }
    }
    stop = clock();
    cout << "MINE_7_3 time usage: " <<  stop - start << endl;
    cout << "filtering count: count1=" << count1 << "  count2=" << count2 << endl;
    //    cout << "ENHLB time: " << stop1-start1 << "  partial HD time: " << totalPHD << "  exact HD time" << totalHD << endl; // you can remove this later
    k = _k;
    for(int i = 0; i < k; i++){
        cout << krecord.top() << endl;
        krecord.pop();
    }
}

//==========================================================================

void testMBRs(PointCloud &ref){
    rtree refRTree(ref.pointcloud.begin(), ref.pointcloud.end());
    vector<RTV::box_type> bounds = getMBRs2(refRTree, 7);
    RTV::box_type box = refRTree.bounds();
    cout << box.m_min_corner.m_values[0] << " " << box.m_min_corner.m_values[1] << " " << box.m_max_corner.m_values[0] << " " << box.m_max_corner.m_values[1] << endl;
    cout << "MBRs from visitor: " << endl;
    for(int i = 0; i < bounds.size(); i++){
        cout << bounds[i].m_min_corner.m_values[0] << " " << bounds[i].m_min_corner.m_values[1] << " " << bounds[i].m_max_corner.m_values[0] << " " << bounds[i].m_max_corner.m_values[1] << endl;
    }
}

//==========================================================================

void KNNSearch::generateBoundsForDataset(string filepath, vector<PointCloud> &dataset){
    ofstream outfile;
    outfile.open(filepath);
    long size1 = dataset.size();
    vector<RTV::box_type> vec;
    for(long i = 0; i < size1; i++){
        rtree dataRtree = rtree(dataset[i].pointcloud.begin(), dataset[i].pointcloud.end());
        RTV::box_type box = dataRtree.bounds();
        outfile << box.m_min_corner.m_values[0] << " " << box.m_min_corner.m_values[1] << " " << box.m_max_corner.m_values[0] << " " << box.m_max_corner.m_values[1] << endl;
        if(i % 1000 == 0){
            cout << i << endl;
        }
    }
}

void KNNSearch::generateMBRsForDataset(string filepath1, string filepath2, vector<PointCloud> &dataset, int number){
    ofstream outfile1, outfile2;
    outfile1.open(filepath1);
    outfile2.open(filepath2);
    
    long size1 = dataset.size();
    vector<RTV::box_type> vec;
    for(long i = 0; i < size1; i++){
        rtree dataRtree = rtree(dataset[i].pointcloud.begin(), dataset[i].pointcloud.end());
        vec.clear();
        vec = getMBRs2(dataRtree, number);
        
        for(int j = 0; j < vec.size(); j++){
            outfile1 << vec[j].m_min_corner.m_values[0] << " " << vec[j].m_min_corner.m_values[1] << " " << vec[j].m_max_corner.m_values[0] << " " << vec[j].m_max_corner.m_values[1] << endl;
        }
        outfile1 << "====" << endl;
        
        RTV::box_type box = dataRtree.bounds();
        outfile2 << box.m_min_corner.m_values[0] << " " << box.m_min_corner.m_values[1] << " " << box.m_max_corner.m_values[0] << " " << box.m_max_corner.m_values[1] << endl;
        
        if(i % 1000 == 0){
            cout << i << endl;
        }
    }
}

// 1 for MBR 2 for bound
void KNNSearch::generateRTreeForDataset(string filepath1, string filepath2, vector<PointCloud> &dataset){
    ofstream outfile1, outfile2;
    outfile1.open(filepath1);
    outfile2.open(filepath2);
    
    long size1 = dataset.size();
    vector<RTV::box_type> vec;
    for(long i = 0; i < size1; i++){
        rtree dataRtree = rtree(dataset[i].pointcloud.begin(), dataset[i].pointcloud.end());
        vec.clear();
        vec = getMBRs(dataRtree);
        
        for(int j = 0; j < vec.size(); j++){ // 如果没有10个的怎么记录？？？会变成两个====连着
            outfile1 << vec[j].m_min_corner.m_values[0] << " " << vec[j].m_min_corner.m_values[1] << " " << vec[j].m_max_corner.m_values[0] << " " << vec[j].m_max_corner.m_values[1] << endl;
        }
        outfile1 << "====" << endl;
        
        RTV::box_type box = dataRtree.bounds();
        outfile2 << box.m_min_corner.m_values[0] << " " << box.m_min_corner.m_values[1] << " " << box.m_max_corner.m_values[0] << " " << box.m_max_corner.m_values[1] << endl;
        
        if(i % 1000 == 0){
            cout << i << endl;
        }
    }
}

// 1 for MBR 2 for bound
void KNNSearch::associateMBRs(string filepath1, string filepath2){
    
    string line;
    int count = 0;
    double minx;
    double miny;
    double maxx;
    double maxy;
    
    // first associate bounds
    ifstream infile2;
    infile2.open(filepath2);
    count = 0;
    vector<string> coordinates;
    while(getline(infile2, line)){
        split(coordinates, line, boost::is_any_of(" "));
        minx = stod(coordinates[0]);
        miny = stod(coordinates[1]);
        maxx = stod(coordinates[2]);
        maxy = stod(coordinates[3]);
        Point point1 = Point(minx, miny);
        Point point2 = Point(maxx, maxy);
        point1.dimension = 2;
        point2.dimension = 2;
        dataset[count].bound = pair<Point, Point>(point1, point2);
        count++;
    }
    
    // then associate MBRs
    count = 0;
    ifstream infile1;
    infile1.open(filepath1);
    vector<pair<Point, Point>> vec;
    while(getline(infile1, line)){
        if(line == "===="){
            if(vec.size() == 0){
                vec.push_back(pair<Point, Point>(dataset[count].bound));
            }
            dataset[count].FirstLevelMBRs = vec;
            vec.clear();
            count++;
            //            cout << "associating..." << count << endl;
        } else {
            split(coordinates, line, boost::is_any_of(" "));
            minx = stod(coordinates[0]);
            miny = stod(coordinates[1]);
            maxx = stod(coordinates[2]);
            maxy = stod(coordinates[3]);
            Point point1 = Point(minx, miny);
            Point point2 = Point(maxx, maxy);
            point1.dimension = 2;
            point2.dimension = 2;
            vec.push_back(pair<Point, Point>(point1, point2));
        }
    }
}

void KNNSearch::generateKeywordId(map<string,int> keywordmap){
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].keywordId = keywordmap[dataset[i].keyword];
    }
    cout << "finish gererating keyword id..." << endl;
}

void KNNSearch::generateKeywordMap(){
    keywordMapForDataset.clear();
    for(int i = 0; i < dataset.size(); i++){
        keywordMapForDataset[dataset[i].keyword] = i;
    }
    cout << "finish gererating keyword map..." << endl;
}

void KNNSearch::generateKeywordIdMap(){
    keywordMapForDataset.clear();
    for(int i = 0; i < dataset.size(); i++){
        keywordIdMapForDataset[dataset[i].keywordId] = i;
    }
    cout << "finish gererating keyword Id map..." << endl;
}

void KNNSearch::generateKeywordCheck(){
    keywordCheck = new bool[dataset.size()];
    std::fill(keywordCheck, keywordCheck+dataset.size(), false);
    cout << "finish gererating keyword check array..." << endl;
}

void KNNSearch::buildRtreeForAllPoints(){
//    allPoints = Dataset::GetAllPoints("/Users/lizhe/Downloads/ICDE15data/Tweets-Character-Lowercase");
    this->refRTree = rtree(allPoints.begin(), allPoints.end());
    cout << "finish building Rtree..." << endl;
}

void KNNSearch::orderDatasetWithSize(){
    sort(dataset.begin(), dataset.end(), cmp_pointcloudsize);
    cout << "finish sorting dataset..." << endl;
}

//========================== GIS2011 =================================


struct cmp_tempResult{
    bool operator()(tempResult t1, tempResult t2){
        return t1.distance > t2.distance;
    }
};

vector<tempResult> KNNSearch::KNN_GIS2011(PointCloud &ref, int k, int number){
    
    vector<tempResult> result;
    priority_queue<tempResult, vector<tempResult>, cmp_tempResult> prqueue;
    
    rtree refRTree(ref.pointcloud.begin(), ref.pointcloud.end());
    
    // calculate ref's bound
    RTV::box_type refbound = refRTree.bounds();
    Point p1(refbound.m_min_corner.m_values[0], refbound.m_min_corner.m_values[1]);
    p1.dimension = 2;
    Point p2(refbound.m_max_corner.m_values[0], refbound.m_max_corner.m_values[1]);
    p2.dimension = 2;
    ref.bound = pair<Point, Point>(p1, p2);
    
    // calculate ref's MBRs
    vector<RTV::box_type> refVec = getMBRs2(refRTree, number); // extract the first level
    vector<pair<Point, Point>> refMBRs;
    for(int i = 0; i < refVec.size(); i++){
        Point p1(refVec[i].m_min_corner.m_values[0], refVec[i].m_min_corner.m_values[1]);
        p1.dimension = 2;
        Point p2(refVec[i].m_max_corner.m_values[0], refVec[i].m_max_corner.m_values[1]);
        p2.dimension = 2;
        refMBRs.push_back(pair<Point, Point>(p1, p2));
    }
    // if there is no first level MBRs
    if(refMBRs.size() == 0){
        refMBRs.push_back(pair<Point, Point>(ref.bound));
    }
    ref.FirstLevelMBRs = refMBRs;
    
    // do randomize for both query and data
    ref.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    clock_t start,stop;
    start = clock();
    
//    clock_t start1, stop1; // can be removed later
//    start1 = clock(); // can be removed later
    
    for(int i = 0; i < dataset.size(); i++){
        tempResult tr(i);
        tr.distance = HausdorffDistanceForBound(ref.bound, dataset[i].bound);
        prqueue.push(tr);
    }
//    stop1 = clock(); // can be removed later
    
    int _k = k;
    
    int count1 = 0, count2 = 0;
    
    // only for counting: !!!!!!!!!!!!!!!!!!!!!!!!! you can remove this later
//    vector<double> exactcount; // you can remove this later
//    bool flag1 = true, flag2 = true, flag3 = true;
    
//    clock_t start2, stop2, start3, stop3, totalENHLB = 0, totalHD = 0; // you can remove this later
    
    long refMBRsize = ref.FirstLevelMBRs.size();
    
    while(k){
        tempResult tr = prqueue.top();
        prqueue.pop();
        if(tr.level == 0){ // consider if there is not multi level
//            start2 = clock(); // you can remove this later
            if(dataset[tr.pcindex].FirstLevelMBRs.size() == 1 && refMBRsize == 1){
                // do nothing
            } else {
                tr.distance = HausdorffDistanceForMBRs(ref.FirstLevelMBRs, dataset[tr.pcindex].FirstLevelMBRs); // this refVec is not enhanced !!!!!!!!!!
            }
            tr.level = 1;
            prqueue.push(tr);
            count1++;
//            stop2 = clock(); // you can remove this later
//            totalENHLB += stop2-start2; // you can remove this later
        } else if(tr.level == 1){
//            start3 = clock(); // you can remove this later
            tr.distance = ExactHausdorff::PAMI2015(ref, dataset[tr.pcindex]);
            tr.level = 2;
            prqueue.push(tr);
            count2++;
//            if(flag1 and tr.distance > 10 and tr.distance < 15){
//                cout << "large DH1: " << tr.distance << endl;
//                dataset[tr.pcindex].storeToFile("/Users/lizhe/Desktop/reports/KNN7/largeDH1");
//                flag1 = false;
//            } else if(flag2 and tr.distance > 15 and tr.distance < 20){
//                cout << "large DH2: " << tr.distance << endl;
//                dataset[tr.pcindex].storeToFile("/Users/lizhe/Desktop/reports/KNN7/largeDH2");
//                flag2 = false;
//            } else if(flag3 and tr.distance > 20){
//                cout << "large DH3: " << tr.distance << endl;
//                dataset[tr.pcindex].storeToFile("/Users/lizhe/Desktop/reports/KNN7/largeDH3");
//                flag3 = false;
//            }
//            exactcount.push_back(tr.distance); // only for counting !!!!!!!!!!!  remove this after finish!!
//            stop3 = clock(); // you can remove this later
//            totalHD += stop3-start3; // you can remove this later
        } else if(tr.level == 2){
            result.push_back(tr);
            k--;
        }
    }
    stop = clock();
    for(int i = 0; i < _k; i++){
        cout << result[i].distance << endl;
    }
    cout << "GIS2011 time usage: " <<  stop - start << endl;
    cout << "filtering count: count1=" << count1 << "  count2=" << count2 << endl;
//    cout << "BscLB time: " << stop1-start1 << "  ENHLB time: " << totalENHLB << "  exact HD time" << totalHD << endl; // you can remove this later
    
//    cout << "exact values " << endl;
//    ofstream outfile;
//    outfile.open("/Users/lizhe/Desktop/reports/KNN7/exact1");
//    for(int i = 0; i < exactcount.size(); i++){
//        outfile << i << " " << exactcount[i] << endl;
//        cout << exactcount[i] << endl;
//    }
//    outfile.close();
//
//    // sort by ascending order
//    sort(exactcount.begin(), exactcount.end());
//
//    outfile.open("/Users/lizhe/Desktop/reports/KNN7/exact2");
//    for(int i = 0; i < exactcount.size(); i++){
//        outfile << i << " " << exactcount[i] << endl;
//        cout << exactcount[i] << endl;
//    }
//    outfile.close();
//
//    cout << "count2 : " << count2 << endl;
//    cout << "exact value size: " << exactcount.size() << endl;
    
    return result;
}

void KNNSearch::KNN_Center(PointCloud &ref, int k){
    
    priority_queue<pair<double, int>, vector<pair<double, int>>, cmp_pair> prqueue;
    ref.calculateCenterPoint();
    // dataset.order by hilbert value
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].calculateCenterPoint();
    }
    
    clock_t start, stop;
    start = clock();
    
    double distance = 0;

    for(int i = 0; i < dataset.size(); i++){
        distance = ref.center.distanceTo(dataset[i].center);
        
        if(prqueue.size() < k){
            prqueue.push(pair<double, int>(distance, i));
        } else if(distance < prqueue.top().first){
            prqueue.pop();
            prqueue.push(pair<double, int>(distance, i));
        }
    }
    
    stop = clock();
    
    ofstream outfile;
    outfile.open("/Users/lizhe/Desktop/reports/final_01/using_Center.csv", ofstream::app);
    outfile << endl;
    
    cout << "using center as distance metric..." << " time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << dataset[prqueue.top().second].keyword << "\t\tdistance: " << prqueue.top().first << "\t\tsize: "<< dataset[prqueue.top().second].pointcloud.size() << endl;
        outfile << dataset[prqueue.top().second].keyword << "," << prqueue.top().first << ","<< dataset[prqueue.top().second].pointcloud.size() << endl;
        prqueue.pop();
    }
}

void KNNSearch::KNN_BHD(PointCloud &ref, int k){
    
    priority_queue<pair<double, int>, vector<pair<double, int>>, cmp_pair> prqueue; // descending
    
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    clock_t start, stop;
    start = clock();
    
    double distance1 = 0, distance2 = 0, distance = 0;
    double kthValue = std::numeric_limits<double>::infinity();
    
    for(int i = 0; i < dataset.size(); i++){
        distance1 = ExactHausdorff::PAMI2015(ref, dataset[i], true, kthValue);
        if(distance1 == -1){
            continue;
        }
        distance2 = ExactHausdorff::PAMI2015(dataset[i], ref, true, kthValue);
        if(distance2 == -1){
            continue;
        }
        
//        if (distance1 == -1 || distance2 == -1){
//            continue;
//        }
        
        distance = max(distance1, distance2);
        
        if(prqueue.size() < k){
            prqueue.push(pair<double, int>(distance, i));
        } else if(distance < prqueue.top().first){
            prqueue.pop();
            prqueue.push(pair<double, int>(distance, i));
            kthValue = prqueue.top().first;
        }
    }
    
    stop = clock();
    
    ofstream outfile;
    outfile.open("/Users/lizhe/Desktop/reports/final_01/using_BHD.csv", ofstream::app);
    outfile << endl;
    cout << "using BHD as distance metric..." << " time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << dataset[prqueue.top().second].keyword << "\t\tdistance: " << prqueue.top().first << "\t\tsize: "<< dataset[prqueue.top().second].pointcloud.size() << endl;
        outfile << dataset[prqueue.top().second].keyword << "," << prqueue.top().first << ","<< dataset[prqueue.top().second].pointcloud.size() << endl;
        prqueue.pop();
    }
}

void KNNSearch::KNN_HDLog(PointCloud &ref, int k){
    
    priority_queue<pair<double, int>, vector<pair<double, int>>, cmp_pair> prqueue; // descending
    
    // dataset.randomize();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    
    clock_t start, stop;
    start = clock();
    
    double distance = 0;
    double kthValue = std::numeric_limits<double>::infinity();
    
    for(int i = 0; i < dataset.size(); i++){
        
        if(dataset[i].keyword == "check"){
            cout << "here" << endl;
        }
        
        distance = ExactHausdorff::PAMI2015_Log(ref, dataset[i], true, kthValue);
//        distance = ExactHausdorff::PAMI2015_Log(ref, dataset[i]);
        if(distance == -1){
            continue;
        }
        if(prqueue.size() < k){
            prqueue.push(pair<double, int>(distance, i));
        } else if(distance < prqueue.top().first){
            prqueue.pop();
            prqueue.push(pair<double, int>(distance, i));
            kthValue = prqueue.top().first;
        }
    }
    
    stop = clock();
    
    ofstream outfile;
    outfile.open("/Users/lizhe/Desktop/reports/final_01/using_HDLog.csv", ofstream::app);
    outfile << endl;
    cout << "using HDLog as distance metric..." << " time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout << dataset[prqueue.top().second].keyword << "\t\tdistance: " << prqueue.top().first << "\t\tsize: "<< dataset[prqueue.top().second].pointcloud.size() << endl;
        outfile << dataset[prqueue.top().second].keyword << "," << prqueue.top().first << ","<< dataset[prqueue.top().second].pointcloud.size() << endl;
        prqueue.pop();
    }
}

//// need to build Rtree and bind keywordMapForDataset first
//void KNNSearch::NN_UsingPoint(PointCloud &ref){ // but searching the nearest one, it should be exactly itself
//
//    clock_t start, stop;
//    start = clock();
//
//    double distance = ExactHausdorff::PAMI2015(ref, dataset[keywordMapForDataset["love"]]);
//
//    long size = ref.pointcloud.size();
//    int randomIndex = 0;
//    srand((unsigned)time(NULL));
//    randomIndex = rand()%size;
//
//    double px = ref.pointcloud[randomIndex].x;
//    double py = ref.pointcloud[randomIndex].y;
//
//    vector<Point> nearbyPoints;
//    box box_region(Point(px-0.5*distance,py-0.5*distance), Point(px+0.5*distance,py+0.5*distance));
//    bgi::query(refRTree, bgi::intersects(box_region), std::back_inserter(nearbyPoints));
//
//    map<string,bool> check;
//    double currentBest = distance;
//    int index = 0;
//
//    for(int i = 0; i < nearbyPoints.size(); i++){
//        for(auto it = nearbyPoints[i].keywords.begin(); it != nearbyPoints[i].keywords.end(); it++){
//            if(check.find(*it) == check.end()){ // unconsidered before
//                index = keywordMapForDataset[*it];
//                distance = ExactHausdorff::PAMI2015(ref, dataset[index], true, currentBest);
//                if(distance == -1){
//                    continue;
//                }
//                if(distance < currentBest){
//                    currentBest = distance;
//                }
//                check[*it] = true;
//            }
//        }
//    }
//
//    stop = clock();
//    cout << "using only points method" << " time usage: " << stop-start << endl;
//    cout << "distance " << currentBest << endl;
//}
//
bool cmp_nearbyPoints(Point &p1, Point &p2){
    return p1.distance < p2.distance;
}
//
//// prerequest:
//// 1. sort dataset according to its size
//// 2. collect all points
//// 3. build Rtree for points
//// 4. build index from keyword to pointcloud-id
//// 5. generateKeywordCheck()
//void KNNSearch::NN_UsingPoint_Efficient(PointCloud &ref){
//
//    ref.sortByKcenter();
//
//    clock_t start, stop;
//    start = clock();
//
//    // calculated upperbound using top 5% of dataset, make sure dataset already sorted
////    int top5 = 0.05*dataset.size();
//    double minupperbound = std::numeric_limits<double>::infinity();
////    double upperbound = 0;
////    for(int i = 0; i < top5; i++){
////        upperbound = ExactHausdorff::PAMI2015(ref, dataset[i]);
////        if(upperbound < minupperbound){
////            minupperbound = upperbound;
////        }
////    }
//
//    for(int i = 0; i < 3; i++){
//        minupperbound = min(minupperbound, ExactHausdorff::PAMI2015(ref, dataset[dataset.size()-100-i])); // distance to love
//    }
//
//    // randomly choose a point for query
//    long size = ref.pointcloud.size();
//    int randomIndex = 0;
//    srand((unsigned)time(NULL));
//    randomIndex = rand()%size;
//
//    double px = ref.pointcloud[randomIndex].x;
//    double py = ref.pointcloud[randomIndex].y;
//
//    vector<Point> nearbyPoints;
//    box box_region(Point(px-minupperbound,py-minupperbound), Point(px+minupperbound,py+minupperbound));
////    box box_region(Point(px-1,py-1), Point(px+1,py+1));
//    bgi::query(refRTree, bgi::intersects(box_region), std::back_inserter(nearbyPoints));
//    for(int i = 0; i < nearbyPoints.size(); i++){
//        nearbyPoints[i].distance = nearbyPoints[i].distanceTo(ref.pointcloud[randomIndex]);
//    }
//    // sort points
//    sort(nearbyPoints.begin(), nearbyPoints.end(), cmp_nearbyPoints); // ascending order
//
//
//    // clear bool array
//    std::fill(keywordCheck, keywordCheck+dataset.size(), false);
//
//    double distance = 0;
//    double currentBest = minupperbound;
//    int pointcloudindex = 0;
//
//    for(int i = 0; i < nearbyPoints.size(); i++){
//        if(nearbyPoints[i].distance >= currentBest){
//            break;
//        }
//        for(auto it = nearbyPoints[i].keywordIds.begin(); it != nearbyPoints[i].keywordIds.end(); it++){
//            if(!keywordCheck[*it]){ // unconsidered before
//                pointcloudindex = keywordIdMapForDataset[*it]; // from keyword-id to point cloud index
//                distance = ExactHausdorff::PAMI2015(ref, dataset[pointcloudindex], true, currentBest);
//                if(distance == -1){
//                    continue;
//                }
//                if(distance < currentBest){
//                    currentBest = distance;
//                }
//                keywordCheck[*it] = true;
//            }
//        }
//    }
//
//    stop = clock();
//    cout << "using only points method" << " time usage: " << stop-start << endl;
//    cout << "distance " << currentBest << endl;
//}
//
//// prerequest:
//// 1. sort dataset according to its size
//// 2. collect all points
//// 3. build Rtree for points
//// 4. build index from keyword to pointcloud-id
//// 5. generateKeywordCheck()
//void KNNSearch::KNN_UsingPoint_Efficient(PointCloud &ref, int k){
//
//    priority_queue<pair<double, int>, vector<pair<double, int>>, cmp_pair> prqueue;
//    ref.sortByKcenter();
//
//    clock_t start, stop;
//    start = clock();
//
//    // here we should use k upper bound and choose the max
//    double maxupperbound = 0;
//    double upperbound = 0;
//    for(int i = 0; i < k; i++){
//        upperbound = ExactHausdorff::PAMI2015(ref, dataset[dataset.size()-50-i]); // from the end
//        if(upperbound > maxupperbound){
//            maxupperbound = upperbound;
//        }
//    }
//
//    // randomly choose a point for query
//    long size = ref.pointcloud.size();
//    int randomIndex = 0;
//    srand((unsigned)time(NULL));
//    randomIndex = rand()%size;
//
//    double px = ref.pointcloud[randomIndex].x;
//    double py = ref.pointcloud[randomIndex].y;
//
//    vector<Point> nearbyPoints;
//    box box_region(Point(px-maxupperbound,py-maxupperbound), Point(px+maxupperbound,py+maxupperbound));
//    bgi::query(refRTree, bgi::intersects(box_region), std::back_inserter(nearbyPoints));
//    for(int i = 0; i < nearbyPoints.size(); i++){
//        nearbyPoints[i].distance = nearbyPoints[i].distanceTo(ref.pointcloud[randomIndex]);
//    }
//    // sort points
//    sort(nearbyPoints.begin(), nearbyPoints.end(), cmp_nearbyPoints); // ascending order
//
//
//    // clear bool array
//    std::fill(keywordCheck, keywordCheck+dataset.size(), false);
//
//    double distance = 0;
//    double kthValue = maxupperbound;
//    int pointcloudindex = 0;
//
//    for(int i = 0; i < nearbyPoints.size(); i++){
//        if(nearbyPoints[i].distance >= kthValue){
//            break;
//        }
//        for(auto it = nearbyPoints[i].keywordIds.begin(); it != nearbyPoints[i].keywordIds.end(); it++){
//            if(!keywordCheck[*it]){ // unconsidered before
//                keywordCheck[*it] = true;
//                pointcloudindex = keywordIdMapForDataset[*it]; // from keyword-id to point cloud index
//                distance = ExactHausdorff::PAMI2015(ref, dataset[pointcloudindex], true, kthValue);
//                if(distance == -1){
//                    continue;
//                }
//                if(prqueue.size() < k){
//                    prqueue.push(pair<double, int>(distance, *it));
//                } else if(distance < prqueue.top().first){
//                    prqueue.pop();
//                    prqueue.push(pair<double, int>(distance, *it));
//                    kthValue = prqueue.top().first;
//                }
//            }
//        }
//    }
//
//    stop = clock();
//    cout << "using only points method" << " time usage: " << stop-start << endl;
//    for(int i = 0; i < k; i++){
//        cout << prqueue.top().first << endl;
//        prqueue.pop();
//    }
//////    cout << "distance " << currentBest << endl;
////    for(int i = 0; i < k; i++){
////        cout << prqueue.top().second << "\t\tdistance: " << prqueue.top().first << "\t\tsize: "<< dataset[keywordIdMapForDataset[prqueue.top().second]].size << endl; // there are some problem
//////        outfile << dataset[prqueue.top().second].keywordId << "," << prqueue.top().first << ","<< dataset[prqueue.top().second].pointcloud.size() << endl;
////        prqueue.pop();
////    }
//}

void KNNSearch::KNN_UsingPoint_Efficient(PointCloud &ref, int k, map<int, vector<int>> &pidKeywordIdsMap, int randomNum){
    priority_queue<pair<double, int>, vector<pair<double, int>>, cmp_pair> prqueue;
    ref.sortByKcenter();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
    }
    srand((unsigned)time(NULL));
    
    clock_t start, stop;
    start = clock();

    clock_t upperboundStart, upperboundEnd;
    upperboundStart = clock();
    // here we should use k upper bound and choose the max
    double maxupperbound = 0;
    double upperbound = 0;
//    for(int i = 0; i < k; i++){
//        upperbound = ExactHausdorff::PAMI2015(ref, dataset[dataset.size()-50-i]); // from the end
//        if(upperbound > maxupperbound){
//            maxupperbound = upperbound;
//        }
//    }
    int top = dataset.size()*0.01;
    priority_queue<double> krecord;
    int randomDatasetIndex = 0;
    for(int i = 0; i < top; i++){
        randomDatasetIndex = rand()%dataset.size();
        upperbound = ExactHausdorff::PAMI2015(ref, dataset[randomDatasetIndex]);
        if(krecord.size() < k){
            krecord.push(upperbound);
        } else if(upperbound < krecord.top()){
            krecord.pop();
            krecord.push(upperbound);
        }
    }
    maxupperbound = krecord.top();
    
    upperboundEnd = clock();
    cout << "max upper bound : " << maxupperbound << endl;
    
    
    // randomly choose some points for query
    
    clock_t queryStart, queryEnd;
    queryStart = clock();
    
    vector<int> randomPositions;
    long size = ref.pointcloud.size();
    int randomIndex = 0;
    for(int i = 0; i < randomNum; i++){
        randomIndex = rand()%size;
        randomPositions.push_back(randomIndex);
    }
    
    // query the nearby points
//    vector<Point> nearbyPoints;
//    vector<Point> minNearbyPoints;
//    double minsize = std::numeric_limits<double>::infinity();
//    double minRandomIndex = 0;
//    for(int i = 0; i < randomNum; i++){
//        double px = ref.pointcloud[randomPositions[i]].x;
//        double py = ref.pointcloud[randomPositions[i]].y;
//        nearbyPoints.clear();
//        box box_region(Point(px-maxupperbound,py-maxupperbound), Point(px+maxupperbound,py+maxupperbound));
//        bgi::query(refRTree, bgi::intersects(box_region), std::back_inserter(nearbyPoints));
//        cout << "nearby points: " << nearbyPoints.size() << endl;
//        if(nearbyPoints.size() < minsize){
//            minsize = nearbyPoints.size();
//            minNearbyPoints = nearbyPoints;
//            minRandomIndex = i;
//        }
//    }
//    for(int i = 0; i < minNearbyPoints.size(); i++){
//        minNearbyPoints[i].distance = minNearbyPoints[i].distanceTo(ref.pointcloud[randomPositions[minRandomIndex]]);
//    }
    
    vector<Point> minNearbyPoints;
    vector<vector<Point>> candidateSets;
    for(int i = 0; i < randomNum; i++){
        double px = ref.pointcloud[randomPositions[i]].x;
        double py = ref.pointcloud[randomPositions[i]].y;
        box box_region(Point(px-maxupperbound,py-maxupperbound), Point(px+maxupperbound,py+maxupperbound));
        bgi::query(refRTree, bgi::intersects(box_region), std::back_inserter(minNearbyPoints));
        cout << "nearby points: " << minNearbyPoints.size() << endl;
        candidateSets.push_back(minNearbyPoints);
    }
    minNearbyPoints.clear();
    doIntersection(candidateSets, minNearbyPoints);
    queryEnd = clock();
    
    // sort points
    sort(minNearbyPoints.begin(), minNearbyPoints.end(), cmp_nearbyPoints); // ascending order
    // clear bool array
    std::fill(keywordCheck, keywordCheck+dataset.size(), false);

    clock_t calculationStart, calculationEnd;
    calculationStart = clock();
    
    double distance = 0;
    double kthValue = maxupperbound;
    int pointcloudindex = 0;
    
    int totalPointClouds = 0;
    int totalPoints = 0;

    for(int i = 0; i < minNearbyPoints.size(); i++){
        if(minNearbyPoints[i].distance >= kthValue){
            cout << "break position: " << i << endl;
            break;
        }
        for(auto it = pidKeywordIdsMap[minNearbyPoints[i].pid].begin(); it != pidKeywordIdsMap[minNearbyPoints[i].pid].end(); it++){
            if(!keywordCheck[*it]){ // unconsidered before
                keywordCheck[*it] = true;
                pointcloudindex = keywordIdMapForDataset[*it]; // from keyword-id to point cloud index
                distance = ExactHausdorff::PAMI2015(ref, dataset[pointcloudindex], true, kthValue);
                totalPointClouds++;
                totalPoints += dataset[pointcloudindex].pointcloud.size();
                if(distance == -1){
                    continue;
                }
                if(prqueue.size() < k){
                    prqueue.push(pair<double, int>(distance, *it));
                } else if(distance < prqueue.top().first){
                    prqueue.pop();
                    prqueue.push(pair<double, int>(distance, *it));
                    kthValue = prqueue.top().first;
                }
            }
        }
    }
    calculationEnd = clock();

    stop = clock();
    cout << "using only points method" << "total time usage: " << stop-start << endl;
    cout << "upperbound time: " << upperboundEnd - upperboundStart << endl;
    cout << "query nearby point time: " << queryEnd - queryStart << endl;
    cout << "calculation time: " << calculationEnd - calculationStart << endl;
    cout << "total consider point clouds " << totalPointClouds << endl;
    cout << "average point cloud size: " << totalPoints/totalPointClouds << endl;
    for(int i = 0; i < k; i++){
        cout << prqueue.top().first << endl;
        prqueue.pop();
    }
}

// ascending order
bool cmp_pid(Point &p1, Point &p2){
    return p1.pid < p2.pid;
}

inline void KNNSearch::doIntersection(vector<vector<Point>> &candidateSets, vector<Point> &candidateSet){
    
    // sort by pid
    for(int i = 0; i < candidateSets.size(); i++){
        sort(candidateSets[i].begin(), candidateSets[i].end(), cmp_pid);
    }
    
    // find min pid
//    int minpid = std::numeric_limits<double>::infinity(); // something wrong
    int minpid = allPoints.size();
    for(int i = 0; i < candidateSets.size(); i++){
        if(candidateSets[i][0].pid < minpid){
            minpid = candidateSets[i][0].pid;
        }
    }
    
    vector<int> currentPosition;
    for(int i = 0; i < candidateSets.size(); i++){
        currentPosition.push_back(0);
    }
    
    // loop to find intersection
    bool findFlag = true;
    bool stopFlag = false;
    while(true){
        findFlag = true;
        for(int i = 0; i < candidateSets.size(); i++){
            while(candidateSets[i][currentPosition[i]].pid < minpid){
                currentPosition[i]++;
                if(currentPosition[i] >= candidateSets[i].size()){
                    stopFlag = true;
                }
            }
            if(candidateSets[i][currentPosition[i]].pid == minpid){ // found a pid not in intersection
                currentPosition[i]++;
                if(currentPosition[i] >= candidateSets[i].size()){
                    stopFlag = true;
                }
            } else if(candidateSets[i][currentPosition[i]].pid > minpid){ // found a pid not in intersection
                findFlag = false;
                //break; // find next one that should be in the result; update the current position
            }
        }
        
        // add the point to result set
        if(findFlag){
            candidateSet.push_back(candidateSets[0][currentPosition[0]]);
        }
        
        // find new min pid
//        minpid = std::numeric_limits<double>::infinity();
        minpid = allPoints.size();
        for(int i = 0; i < candidateSets.size(); i++){
            if(candidateSets[i][currentPosition[i]].pid < minpid){
                minpid = candidateSets[i][currentPosition[i]].pid;
            }
        }
        
        // see if stop
        if(stopFlag){
            break;
        }
    }
}

#endif /* KNNSearch_hpp */
