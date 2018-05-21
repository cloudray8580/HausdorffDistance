//
//  BoostRTreeSetting.hpp
//  HausdorffDistance
//
//  Created by 李喆 on 2018/3/8.
//  Copyright © 2018年 李喆. All rights reserved.
//

#ifndef BoostRTreeSetting_hpp
#define BoostRTreeSetting_hpp

#include <stdio.h>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <queue> // used for priority_queue

//#include "PointCloud.hpp" // do not do this, cause you have include this in pointcloud
//#include "KNNSearch.hpp"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
namespace bgid = bgi::detail;
typedef bgi::rtree<Point, bgi::linear<10>> rtree;
typedef bg::model::point<float, 2, bg::cs::cartesian> point;
typedef bg::model::box<Point> box;
typedef bgid::rtree::utilities::view<rtree> RTV;
BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(Point, double, bg::cs::spherical_equatorial<bg::degree>, getX, getY, setX, setY)

using namespace std;

struct tempResult{
    int pcindex;
    double distance;
    int level;
    tempResult(int index){
        pcindex = index;
        level = 0;
    }
};

double HausdorffDistanceForBound(pair<Point, Point> &refBound, pair<Point, Point> &bound);
double HausdorffDistanceForMBRs(vector<pair<Point,Point>> &refMBRs, vector<pair<Point,Point>> &MBRs);
double distanceFromEdgeToMBR(double &edgeminx, double &edgeminy, double &edgemaxx, double &edgemaxy, double &mbrminx, double &mbrminy, double &mbrmaxx, double &mbrmaxy);
double distanceFromPointToMBR(Point& p, pair<Point, Point> &bound);
vector<RTV::box_type> getMBRs(rtree &RTree);
vector<RTV::box_type> getMBRsWithNumber(rtree &RTree, int number);
vector<RTV::box_type> getMBRs2(rtree &RTree, int number);

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
        
        //        cout << "element size: " << elements.size() << endl;
        
        for ( typename elements_type::const_iterator it = elements.begin();
             it != elements.end() ; ++it)
        {
            handle_box_or_value(it->first); // now we only extract the first level MBRs
            //            bgid::rtree::apply_visitor(*this, *(it->second));
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
        //        std::cout << bg::dsv(b) << std::endl;
    }
};

// ==========================================

// if not enough MBRs, return all MBRs
// if enough, return at least "number" MBRs
template <typename Value,typename Options, typename Translator, typename Box, typename Allocators> class breadth_first_visitor
: public bgid::rtree::visitor<Value, typename Options::parameters_type, Box, Allocators, typename Options::node_tag, true>::type

{
    typedef typename bgid::rtree::internal_node<Value, typename Options::parameters_type, Box, Allocators, typename Options::node_tag>::type internal_node;
    typedef typename bgid::rtree::leaf<Value, typename Options::parameters_type, Box, Allocators, typename Options::node_tag>::type leaf;
    typedef typename Allocators::node_pointer node_pointer;
    
public:
    void operator()(internal_node const& n){
        typedef typename bgid::rtree::elements_type<internal_node>::type elements_type;
        elements_type const& elements = bgid::rtree::elements(n);
        
        if(firstFlag){
            firstFlag = false;
        } else {
            MBRs.pop_front();
            //            cout << "MBRs pop front" << endl;
        }
        
        for ( typename elements_type::const_iterator it = elements.begin(); it != elements.end() ; ++it){
            // do something with MBR
            //            std::cout << bg::dsv(it->first) << std::endl;
            MBRs.push_back(it->first); // 没有remove 掉 pop 出来的 MBR ！！！！！！！！！！！！！
            // add child node pointer to the container
            m_nodes.push_back(it->second);
        }
        if(MBRs.size() < number){
            traverse();
        }
    }
    
    void operator()(leaf const& n){
        //        std::cout << boost::size(bgid::rtree::elements(n)) << " elements in leaf" << std::endl;
        //traverse();
    }
    
    void traverse(){
        if (! m_nodes.empty()){
            node_pointer next = m_nodes.front();
            m_nodes.pop_front();
            //            MBRs.pop_front();
            bgid::rtree::apply_visitor(*this, *next);
        }
    }
    
    std::deque<node_pointer> m_nodes;
    std::deque<Box> MBRs; // 也改成deque ? 然后同步pop？
    int number;
    bool firstFlag = true;
};

vector<RTV::box_type> getMBRs2(rtree &RTree, int number){
    
    RTV Rtv(RTree);
    
    breadth_first_visitor<
    typename RTV::value_type,
    typename RTV::options_type,
    typename RTV::translator_type,
    typename RTV::box_type,
    typename RTV::allocators_type
    > visitor;
    
    visitor.number = number;
    
    Rtv.apply_visitor(visitor);
    
    deque<RTV::box_type> _MBRs = visitor.MBRs;
    vector<RTV::box_type> MBRs;
    while(!_MBRs.empty()){
        MBRs.push_back(_MBRs.front());
        _MBRs.pop_front();
    }
    return MBRs;
}

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

// calculate the hausdorff distance for bounds
double HausdorffDistanceForBound(pair<Point, Point> &refBound, pair<Point, Point> &bound){
    double distance1 = std::numeric_limits<double>::infinity(); // left edge to box, use minx, maxx = minx
    double distance2 = std::numeric_limits<double>::infinity(); // upper edge to box, use maxy, miny = maxy
    double distance3 = std::numeric_limits<double>::infinity(); // right edge to box, use maxx, minx = minx
    double distance4 = std::numeric_limits<double>::infinity(); // down edge to box, use miny, maxy = miny
    double HausdorffDistance = 0;
    distance1 = distanceFromEdgeToMBR(refBound.first.x, refBound.first.y, refBound.first.x, refBound.second.y, bound.first.x, bound.first.y, bound.second.x, bound.second.y);
    distance2 = distanceFromEdgeToMBR(refBound.first.x, refBound.second.y, refBound.second.x, refBound.second.y, bound.first.x, bound.first.y, bound.second.x, bound.second.y);
    distance3 = distanceFromEdgeToMBR(refBound.second.x, refBound.first.y, refBound.second.x, refBound.second.y, bound.first.x, bound.first.y, bound.second.x, bound.second.y);
    distance4 = distanceFromEdgeToMBR(refBound.first.x, refBound.first.y, refBound.second.x, refBound.first.y, bound.first.x, bound.first.y, bound.second.x, bound.second.y);
    HausdorffDistance = distance1; // you left this!
    if(distance2 >= HausdorffDistance){
        HausdorffDistance = distance2; // a mistake here
    }
    if(distance3 >= HausdorffDistance){
        HausdorffDistance = distance3;
    }
    if(distance4 >= HausdorffDistance){
        HausdorffDistance = distance4;
    }
    return HausdorffDistance;
}

double HausdorffDistanceForMBRs(vector<pair<Point,Point>> &refMBRs, vector<pair<Point,Point>> &MBRs){
    double distance1 = std::numeric_limits<double>::infinity(); // left edge to box, use minx, maxx = minx
    double distance2 = std::numeric_limits<double>::infinity(); // upper edge to box, use maxy, miny = maxy
    double distance3 = std::numeric_limits<double>::infinity(); // right edge to box, use maxx, minx = minx
    double distance4 = std::numeric_limits<double>::infinity(); // down edge to box, use miny, maxy = miny
    double HausdorffDistance = 0;
    for(int i = 0; i < refMBRs.size(); i++){
        distance1 = std::numeric_limits<double>::infinity();
        distance2 = std::numeric_limits<double>::infinity();
        distance3 = std::numeric_limits<double>::infinity();
        distance4 = std::numeric_limits<double>::infinity();
        for(int j = 0; j < MBRs.size(); j++){
            distance1 = min(distance1, distanceFromEdgeToMBR(refMBRs[i].first.x, refMBRs[i].first.y, refMBRs[i].first.x, refMBRs[i].second.y, MBRs[j].first.x, MBRs[j].first.y, MBRs[j].second.x, MBRs[j].second.y));
            distance2 = min(distance2, distanceFromEdgeToMBR(refMBRs[i].first.x, refMBRs[i].second.y, refMBRs[i].second.x, refMBRs[i].second.y, MBRs[j].first.x, MBRs[j].first.y, MBRs[j].second.x, MBRs[j].second.y));
            distance3 = min(distance3, distanceFromEdgeToMBR(refMBRs[i].second.x, refMBRs[i].first.y, refMBRs[i].second.x, refMBRs[i].second.y, MBRs[j].first.x, MBRs[j].first.y, MBRs[j].second.x, MBRs[j].second.y));
            distance4 = min(distance4, distanceFromEdgeToMBR(refMBRs[i].first.x, refMBRs[i].first.y, refMBRs[i].second.x, refMBRs[i].first.y, MBRs[j].first.x, MBRs[j].first.y, MBRs[j].second.x, MBRs[j].second.y));
        }
        if(distance1 > HausdorffDistance){
            HausdorffDistance = distance1;
        }
        if(distance2 > HausdorffDistance){
            HausdorffDistance = distance2;
        }
        if(distance3 > HausdorffDistance){
            HausdorffDistance = distance3;
        }
        if(distance4 > HausdorffDistance){
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

double distanceFromPointToMBR(Point& p, pair<Point, Point> &bound){
    
    double distance;
    
    double mbrminx, mbrminy, mbrmaxx, mbrmaxy;
    mbrminx = bound.first.x;
    mbrminy = bound.first.y;
    mbrmaxx = bound.second.x;
    mbrmaxy = bound.second.y;
    
    bool horizontal_valid = false;
    bool vertical_valid = false;
    if(p.x < mbrminx || p.x > mbrmaxx){
        horizontal_valid = true;
    }
    if(p.y < mbrminy || p.y > mbrmaxy){
        vertical_valid = true;
    }
    if(!horizontal_valid && !vertical_valid){
        distance = 0;
    } else if(horizontal_valid && !vertical_valid){
        distance = min(fabs(p.x-mbrminx), fabs(p.x-mbrmaxx));
    } else if(vertical_valid && !horizontal_valid){
        distance = min(fabs(p.y-mbrminy), fabs(p.y-mbrmaxy));
    } else {
        double tempx = min(fabs(p.x-mbrminx), fabs(p.x-mbrmaxx));
        double tempy = min(fabs(p.y-mbrminy), fabs(p.y-mbrmaxy));
        distance = sqrt(tempx*tempx + tempy*tempy);
    }
    
    return distance;
}

#endif /* BoostRTreeSetting_hpp */
