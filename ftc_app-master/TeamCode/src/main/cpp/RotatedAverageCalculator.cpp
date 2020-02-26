//
// Created by peter on 2/28/19.
//

#include <opencv2/imgproc.hpp>
#include "RotatedAverageCalculator.h"
using namespace cv;

/**
 * Get's the average of a rotated row in an image
 * @param img the mat of the image
 * @param angle_rad angle that we will average at (0 degrees = horizontal)
 * @return a scalar of the color
 */
Vec3b* RotatedAverageCalculator::getLineAverage(Mat* img, int y, double angle_rad,
                                                int leftBound,int rightBound,
                                                bool draw,Mat* drawingImage) {
    //we will rotate how we look at the image about this point. For now it is in the center
    //and at the row the user specifies
    int centerOfRotationX = img->cols/2;


    //these are sums of the three parts of the scalar. Start of at 0
    int SumR = 0;
    int SumG = 0;
    int SumB = 0;
    int totalPixels = 0;
    //now go from left to right on the image, getting the sum
    for(int x = leftBound; x <= rightBound; x ++){
        //we can use trig to solve for this y


        //                                         -----
        //   x-centerOfRotationX              -----
        //  |---------------------------x---------------
        //  ||              A-----
        //  |y         -----
        //  ||    -----
        //  |-----



        int thisY = (int) ((x-centerOfRotationX)*tan(angle_rad)) + y;
        if(thisY < 0 || thisY >= img->rows){continue;}//continue if we are out of bounds


        if(draw){
            circle(*drawingImage,Point(x*4,thisY*4),3,Scalar(255,0,0),-1);
        }
        SumR += img->at<Vec3b>(y,x)[0];
        SumG += img->at<Vec3b>(y,x)[1];
        SumB += img->at<Vec3b>(y,x)[2];
        totalPixels ++;//only increment if we are in bounds of the image
    }

    //return the average color
    return new Vec3b(SumR/totalPixels,SumG/totalPixels,SumB/totalPixels);
}
