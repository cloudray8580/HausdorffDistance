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
#endif /* Comparison_hpp */
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
