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

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <queue> // used for priority_queue
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
namespace bgid = bgi::detail;
typedef bgi::rtree<Point, bgi::linear<100>> rtree;
BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(Point, double, bg::cs::spherical_equatorial<bg::degree>, getX, getY, setX, setY)

struct tempResult{
    PointCloud pointcloud;
    rtree RTree;
    double distance;
    int level;
    tempResult(PointCloud &pc){
        pointcloud = pc;
    }
};

class KNNSearch{
public:
    KNNSearch(){this->dataset.clear();};
    KNNSearch(string directory);
    
    vector<PointCloud> dataset;
    
    void loadDataset(string directory);
    
    vector<pair<double,PointCloud>> KNN_PAMI2015(PointCloud &ref, int k);
    vector<pair<double,PointCloud>> KNN_PR2017(PointCloud &ref, int k);
    vector<tempResult> KNN_GIS2011(PointCloud &ref, int k);
    
    void KNN_PAMI2015_Pruning(PointCloud &ref, int k); // use a priority queue to keep K
    void KNN_PR2017_Pruning(PointCloud &ref, int k); // use a priority queue to keep K
    
    void Test_Time_KNN_PAMI2015_Pruning(PointCloud &ref, int k);
    void Test_Time_KNN_PAMI2015_Pruning2(PointCloud &ref, int k);
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

void KNNSearch::KNN_PAMI2015_Pruning(PointCloud &ref, int k){
    
    clock_t start, stop;
    start = clock();
    priority_queue<double, vector<double>, cmp_double> prqueue;
    
    ref.randomize();
    int count = 1;
    for(int i = 0; i < dataset.size(); i++){
        dataset[i].randomize();
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
    cout << "PAMI2015 time usage: " << stop-start << endl;
    for(int i = 0; i < k; i++){
        cout<< prqueue.top() << endl;
        prqueue.pop();
    }
}

bool cmp_pointcloudsize(PointCloud &pc1, PointCloud &pc2){
    return pc1.pointcloud.size() < pc2.pointcloud.size();
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

//============= the below is about the GIS2011 method ===================


template <typename Value, typename Options, typename Translator, typename Box, typename Allocators>
class test_visitor
: public bgid::rtree::visitor<Value, typename Options::parameters_type, Box, Allocators, typename Options::node_tag, true>::type
{
    typedef typename bgid::rtree::internal_node<Value, typename Options::parameters_type, Box, Allocators, typename Options::node_tag>::type internal_node;
    typedef typename bgid::rtree::leaf<Value, typename Options::parameters_type, Box, Allocators, typename Options::node_tag>::type leaf;
    
public:
    vector<Box> MBRs;
    void operator()(internal_node const& n)
    {
        typedef typename bgid::rtree::elements_type<internal_node>::type elements_type;
        elements_type const& elements = bgid::rtree::elements(n);

        for ( typename elements_type::const_iterator it = elements.begin();
             it != elements.end() ; ++it)
        {
            handle_box_or_value(it->first);

            bgid::rtree::apply_visitor(*this, *(it->second));
        }
    }
    
    // that is for print the leaf value, e.g., points, instead of MBRs
//    void operator()(leaf const& n)
//    {
//        typedef typename bgid::rtree::elements_type<leaf>::type elements_type;
//        elements_type const& elements = bgid::rtree::elements(n);
//
//        for ( typename elements_type::const_iterator it = elements.begin();
//             it != elements.end() ; ++it)
//        {
//            handle_box_or_value(*it);
//        }
//    }
    
    template <typename BoxOrValue>
    void handle_box_or_value(BoxOrValue const& b)
    {
        MBRs.push_back(b);
        std::cout << bg::dsv(b) << std::endl;
    }
};

typedef bg::model::point<float, 2, bg::cs::cartesian> point;
typedef bg::model::box<Point> box;
//typedef bgi::rtree<Point, bgi::linear<3>> rtree;
typedef bgid::rtree::utilities::view<rtree> RTV;

struct cmp_tempResult{
    bool operator()(tempResult t1, tempResult t2){
        return t1.distance > t2.distance;
    }
};

double HausdorffDistanceForMBRs(vector<RTV::box_type> &refMBRs, vector<RTV::box_type> &MBRs);
double distanceFromEdgeToMBR(double &edgeminx, double &edgeminy, double &edgemaxx, double &edgemaxy, double &mbrminx, double &mbrminy, double &mbrmaxx, double &mbrmaxy);
vector<RTV::box_type> getMBRs(rtree &RTree);


vector<tempResult> KNNSearch::KNN_GIS2011(PointCloud &ref, int k){
    
    clock_t start,stop;
    start = clock();
    vector<tempResult> result;
    priority_queue<tempResult, vector<tempResult>, cmp_tempResult> prqueue;
    
    rtree refRTree(ref.pointcloud.begin(), ref.pointcloud.end());
    vector<RTV::box_type> refVec, vec;
    refVec.push_back(refRTree.bounds());
    double tempHausdorffDistance = 0;
    for(int i = 0; i < dataset.size(); i++){
        tempResult tr(dataset[i]);
        tr.RTree = rtree(dataset[i].pointcloud.begin(), dataset[i].pointcloud.end());
        vec.clear();
        vec.push_back(tr.RTree.bounds());
        tr.distance = HausdorffDistanceForMBRs(refVec, vec);
        tr.level = 0;
        prqueue.push(tr);
        if(i % 1000 == 0){
            cout << "building RTree " << i << endl;
        }
    }
    
    while(k){
        tempResult tr = prqueue.top();
        prqueue.pop();
        if(tr.level == 0){ // consider if there is not multi level
            vec.clear();
            vec = getMBRs(tr.RTree);
            if(vec.size() == 0){
                // do nothing
            } else {
                tr.distance = HausdorffDistanceForMBRs(refVec, vec);
            }
            tr.level = 1;
            prqueue.push(tr);
        } else if(tr.level == 1){
            tr.distance = ExactHausdorff::PAMI2015(ref, tr.pointcloud);
            tr.level = 2;
            prqueue.push(tr);
        } else if(tr.level == 2){
            result.push_back(tr);
            k--;
        }
    }
    stop = clock();
//    for(int i = 0; i < 10; i++){
//        cout << result[i].distance << endl;
//    }
    cout << stop - start << endl;
    return result;
}

/*
void KNNSearch::KNN_GIS2011(PointCloud &ref, int k){
    vector<Point> points = ref.pointcloud;
    rtree refRTree(points.begin(), points.end());
//    bgi::rtree<Point, bgi::linear<10>>::bounds_type bounds = RTree.bounds();
    
    RTV refRtv(refRTree);
    
    test_visitor<
    typename RTV::value_type,
    typename RTV::options_type,
    typename RTV::translator_type,
    typename RTV::box_type,
    typename RTV::allocators_type
    > refVisitor, visitor;
    
    refRtv.apply_visitor(refVisitor);
    
    // test if there are only root node. or the below value is none
    vector<RTV::box_type> refMBRs = refVisitor.MBRs;
    vector<RTV::box_type> MBRs;
    if(refMBRs.size() == 0){
        
    } else {
//        rtree RTree(dataset[1].pointcloud.begin(),dataset[i].pointcloud.end());
//        RTV rtv(RTree);
//        rtv.apply_visitor(visitor);
//        MBRs.clear();
//        MBRs = visitor.MBRs;
//
    }


    // cout bounds in well known text
//    cout << bg::wkt<bgi::rtree<Point, bgi::linear<10>>::bounds_type>(bounds) << endl;
    
    // get all points in rtree
//    cout << bg::wkt<point>(point(0,0)) << endl;
//    for(auto it:RTree){
//        cout << it.x << endl;
//    }
    
    // process query
//    box querybox(Point(0,0),Point(0.1,2));
//    vector<Point> resultset;
//    RTree.query(bgi::intersects(querybox), std::back_inserter(resultset));
//    for(auto it:resultset){
//        cout << it.x << endl;
//    }
    
    // get all points count
//    cout << RTree.size() << endl;
    
}
 */

vector<RTV::box_type> getMBRs(rtree &RTree){
    
    RTV Rtv(RTree);
    
    test_visitor<
    typename RTV::value_type,
    typename RTV::options_type,
    typename RTV::translator_type,
    typename RTV::box_type,
    typename RTV::allocators_type
    > visitor;
    
    Rtv.apply_visitor(visitor);
    return visitor.MBRs;
}


// return the Hausdorff distance from the reference pointcloud's MBRs to the other pointcloud's MBRs
double HausdorffDistanceForMBRs(vector<RTV::box_type> &refMBRs, vector<RTV::box_type> &MBRs){

    double distance1 = std::numeric_limits<double>::infinity(); // left edge to box, use minx, maxx = minx
    double distance2 = std::numeric_limits<double>::infinity(); // upper edge to box, use maxy, miny = maxy
    double distance3 = std::numeric_limits<double>::infinity(); // right edge to box, use maxx, minx = minx
    double distance4 = std::numeric_limits<double>::infinity(); // down edge to box, use miny, maxy = miny
    double HausdorffDistance = 0;

    for(int i = 0; i < refMBRs.size(); i++){
        for(int j = 0; j < MBRs.size(); j++){
            distance1 = min(distance1, distanceFromEdgeToMBR(refMBRs[i].m_min_corner.m_values[0], refMBRs[i].m_min_corner.m_values[1], refMBRs[i].m_min_corner.m_values[0], refMBRs[i].m_max_corner.m_values[1], MBRs[j].m_min_corner.m_values[0], MBRs[j].m_min_corner.m_values[1], MBRs[j].m_max_corner.m_values[0], MBRs[j].m_max_corner.m_values[1]));
            distance2 = min(distance1, distanceFromEdgeToMBR(refMBRs[i].m_min_corner.m_values[0], refMBRs[i].m_max_corner.m_values[1], refMBRs[i].m_min_corner.m_values[0], refMBRs[i].m_max_corner.m_values[1], MBRs[j].m_min_corner.m_values[0], MBRs[j].m_min_corner.m_values[1], MBRs[j].m_max_corner.m_values[0], MBRs[j].m_max_corner.m_values[1]));
            distance3 = min(distance1, distanceFromEdgeToMBR(refMBRs[i].m_max_corner.m_values[0], refMBRs[i].m_min_corner.m_values[1], refMBRs[i].m_min_corner.m_values[0], refMBRs[i].m_max_corner.m_values[1], MBRs[j].m_min_corner.m_values[0], MBRs[j].m_min_corner.m_values[1], MBRs[j].m_max_corner.m_values[0], MBRs[j].m_max_corner.m_values[1]));
            distance4 = min(distance1, distanceFromEdgeToMBR(refMBRs[i].m_min_corner.m_values[0], refMBRs[i].m_min_corner.m_values[1], refMBRs[i].m_min_corner.m_values[0], refMBRs[i].m_min_corner.m_values[1], MBRs[j].m_min_corner.m_values[0], MBRs[j].m_min_corner.m_values[1], MBRs[j].m_max_corner.m_values[0], MBRs[j].m_max_corner.m_values[1]));
        }
        HausdorffDistance = distance1;
        if(distance2 >= HausdorffDistance){
            HausdorffDistance = distance1;
        }
        if(distance3 >= HausdorffDistance){
            HausdorffDistance = distance3;
        }
        if(distance4 >= HausdorffDistance){
            HausdorffDistance = distance4;
        }
    }
    return HausdorffDistance;
}

// calculate the distance from an edge to a single MBR
double distanceFromEdgeToMBR(double &edgeminx, double &edgeminy, double &edgemaxx, double &edgemaxy, double &mbrminx, double &mbrminy, double &mbrmaxx, double &mbrmaxy){
    
    double distance = 0;
    bool horizontal_valid = false;
    bool vertical_valid = false;
    
    if(edgeminx > mbrmaxx || edgemaxx < mbrminx){ // non overlap in horizontal
        horizontal_valid = true;
    }
    if(edgeminy > mbrmaxy || edgemaxy < mbrminy){ // non overlap in vertical
        vertical_valid = true;
    }
    
    if (!horizontal_valid && !vertical_valid){
        distance = 0;
    } else if(horizontal_valid && !vertical_valid){
        distance = min(fabs(edgeminx - mbrmaxx), fabs(edgemaxx - mbrminx));
    } else if(!horizontal_valid && vertical_valid){
        distance = min(fabs(edgeminy - mbrmaxy), fabs(edgemaxy - mbrminy));
    } else if(horizontal_valid && vertical_valid){
        double tempdisx = min(fabs(edgeminx-mbrmaxx), fabs(edgemaxx-mbrminx));
        double tempdisy = min(fabs(edgeminy-mbrmaxy), fabs(edgemaxy-mbrminy));
        distance = sqrt(tempdisx*tempdisx + tempdisy*tempdisy);
    } // else if inner mbr, the distance is 0
    
    return distance;
}

#endif /* KNNSearch_hpp */
