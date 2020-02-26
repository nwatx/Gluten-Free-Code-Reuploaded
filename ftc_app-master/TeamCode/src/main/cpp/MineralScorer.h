//
// Created by peter on 2/28/2019.
//

#ifndef ROVERRUCKUS_MINERALSCORER_H
#define ROVERRUCKUS_MINERALSCORER_H


#include <opencv2/core.hpp>
#include <android/log.h>
#include "CollectingLocation.h"

using namespace cv;
using namespace std;
/**
 * MineralScorer.h
 * This class is designed to take raw pixel data about the minerals and process
 * it into a robot path.
 */
class MineralScorer {
public:
    const double PI = 3.1415926536;

    //These are the robot coordinates and need to be updated
    double robotX = 138;
    double robotY = 138;
    double robotAngle = -0.75 * PI;//start at -135 degrees


    //An array of field coordinates of all the mineral pixels
    vector<Point2d> mineralPoints = vector<Point2d>();




    //the x size of the craterImage
    static double RESOLUTION_X;
    //the y size of the craterImage
    static double RESOLUTION_Y;

    //this is the field size of the craterImage in cm
    static double CRATER_IMAGE_SIZE_X;
    static double CRATER_IMAGE_SIZE_Y;


    static double CRATER1_TOP_LEFT_X;
    static double CRATER1_TOP_LEFT_Y;
    static double CRATER1_BOTTOM_RIGHT_X;
    static double CRATER1_BOTTOM_RIGHT_Y;





    //This is an image in WORLD SCALE where the origin of the image is the origin of the field
    Mat craterImage = Mat(RESOLUTION_Y,RESOLUTION_X,CV_8UC1,Scalar(0));


    //use this to draw on the phone image
    Mat* drawingImage;


    //just set's our remembered robot position
    void updateRobotPosition(double robotX, double robotY, double robotAngle);
    //converts a screen coordinate to a field coordinate
    Point2d screenToField(int x, int y, int widthScreen, int heightScreen);

    static Point pointToImage(Point2d point);
    Point2d imagePointToWorld(Point point);


    void updateMinerals(Mat* binary);


    CollectingLocation* findBestPath();



    vector<Point2d>* lineCircleIntersection(double circleX, double circleY, double r,
                                           double lineX1, double lineY1,
                                           double lineX2, double lineY2);


    Point2d* lineIntersection(Point2d point1,double m1, Point2d point2, double m2);



    void scoreImageLocation(CollectingLocation* collectingLocation);



    static double angleWrap(double angle);
};


#endif //ROVERRUCKUS_MINERALSCORER_H
