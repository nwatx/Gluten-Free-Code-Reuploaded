//
// Created by peter on 3/8/2019.
//

#ifndef ROVERRUCKUS_COLLECTINGLOCATION_H
#define ROVERRUCKUS_COLLECTINGLOCATION_H

#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

class CollectingLocation {
public:
    CollectingLocation();
    //dropZone the location of the drop zone in world space
    Point2d dropZone;
    Point dropZoneScreen;
    //the location  we are collecting from
    Point2d collectingLocation;
    Point collectingLocationScreen;


    //our score, bigger is better
    double score = 0;

    //use this to set the collecting location
    void setCollectingLocation(Point2d worldCoordinate);


    void setDropZone(Point2d dropZone);




    void setScore(double score);
};


#endif //ROVERRUCKUS_COLLECTINGLOCATION_H
