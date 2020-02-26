//
// Created by peter on 2/28/19.
//

#ifndef MASTER_ROTATEDAVERAGECALCULATOR_H
#define MASTER_ROTATEDAVERAGECALCULATOR_H

#include <opencv2/core.hpp>

using namespace cv;

class RotatedAverageCalculator {
public:
    Vec3b* getLineAverage(Mat*,int,double,int,int,bool,Mat*);
};


#endif //MASTER_ROTATEDAVERAGECALCULATOR_H
