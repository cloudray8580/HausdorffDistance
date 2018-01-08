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

#endif /* KNNSearch_hpp */

class KNNSearch{
public:
    KNNSearch(string directory);
    
    vector<PointCloud> dataset;
    
    void loadDataset(string directory);
    
    vector<pair<double,PointCloud>> KNN_PAMI2015(PointCloud ref, int k);
    vector<pair<double,PointCloud>> KNN_PR2017(PointCloud ref, int k);
};

KNNSearch::KNNSearch(string directory){
    loadDataset(directory);
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
bool cmp_hausdorff(pair<double,PointCloud> p1, pair<double,PointCloud> p2){
    return p1.first < p2.first;
}

vector<pair<double,PointCloud>> KNNSearch::KNN_PAMI2015(PointCloud ref, int k){
    vector<pair<double,PointCloud>> all;
    double EHD = 0;
    for(int i = 0; i < dataset.size(); i++){
        EHD = ExactHausdorff::PAMI2015(ref, dataset[i]);
        all.push_back(pair<double, PointCloud>(EHD, dataset[i]));
    }
    sort(all.begin(),all.end(),cmp_hausdorff);
    vector<pair<double,PointCloud>> result(all.begin(), all.begin()+k);
    return result;
}

vector<pair<double,PointCloud>> KNNSearch::KNN_PR2017(PointCloud ref, int k){
    vector<pair<double,PointCloud>> all;
    double EHD = 0;
    ref.prezorder();
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].prezorder();
        EHD = ExactHausdorff::PR2017(ref, dataset[i]);
        all.push_back(pair<double, PointCloud>(EHD, dataset[i]));
    }
    sort(all.begin(),all.end(),cmp_hausdorff);
    vector<pair<double,PointCloud>> result(all.begin(), all.begin()+k);
    return result;
}
