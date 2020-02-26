//
// Created by peter on 2/28/2019.
//

#include <opencv2/imgproc.hpp>
#include "MineralScorer.h"
#include "CollectingLocation.h"
#include <android/log.h>

double MineralScorer::RESOLUTION_X = 75;
double MineralScorer::RESOLUTION_Y = 75;

double MineralScorer::CRATER_IMAGE_SIZE_X = 131;
double MineralScorer::CRATER_IMAGE_SIZE_Y = 161;

double MineralScorer::CRATER1_TOP_LEFT_X = 0;
double MineralScorer::CRATER1_TOP_LEFT_Y = 160;

double MineralScorer::CRATER1_BOTTOM_RIGHT_X = 130;
double MineralScorer::CRATER1_BOTTOM_RIGHT_Y = 0;


/**
 * Converts a screen coordinate to a field coordinate
 * @param x pixel coordinate
 * @param y pixel coordinate
 * @return the real coordinate
 */
Point2d MineralScorer::screenToField(int x, int y,int widthScreen,int heightScreen) {
    //first get the screen percent coordinates of the point
    double xPercent = (double) x/widthScreen;
    double yPercent = (double) y/heightScreen;

    //now we can get the relative y position of the point
    double yRelativeCm = -271*pow(yPercent,3) + 694*pow(yPercent,2) + -683*yPercent + 305 - 7;


    //get the width of the screen in cm (see desmos for this)
    double screenWidthCm = -863*pow(yPercent,3)+1777*pow(yPercent,2)-1318*yPercent+474;
    //now we can get the relative x position of the point (0 when percent is 0.5)
    double xRelativeCm = (xPercent - 0.5) * screenWidthCm;
    xRelativeCm -=5;


    //now calculate how much we need to add to our angle to get the angle to the point
    double addToAngle = atan2(-xRelativeCm,yRelativeCm);
    //calculate the total distance away of the point
    double distance = hypot(xRelativeCm,yRelativeCm);

    //now we can get the field coordinates of the pixel
    double xField = (distance * cos(robotAngle + addToAngle)) + robotX;
    double yField = (distance * sin(robotAngle + addToAngle)) + robotY;

    //now we can return the new point
    return cv::Point2d(xField,yField);
}

/**
 * Call this to update the stored robot position
 * @param robotX field position x in cm
 * @param robotY field position y in cm
 * @param robotAngle angle in radians
 */
void MineralScorer::updateRobotPosition(double robotX, double robotY, double robotAngle) {
    this->robotX = robotX;
    this->robotY = robotY;
    this->robotAngle = robotAngle;
}

/**
 * This method converts a binary mat of screen location of minerals to actual coordinates
 * It then condenses
 * @param binary Mat* to the binary mat
 */
void MineralScorer::updateMinerals(Mat *binary) {

    //we first have to clear the mineral points
    mineralPoints.clear();

    //reset the crater image since this is a new scan
    craterImage.setTo(Scalar(0));

    //go through the entire image and populate the array
    for(int y = 0; y < binary->rows; y+=1){
        for(int x = 0; x < binary->cols; x +=1){
            //this is a mineral if the binary is not 0
            bool isMineral = binary->at<uchar>(y,x) != 0;
            if(isMineral){
                //convert it to field coordinates
                Point2d thisPoint = Point2d(screenToField(x,y,
                        binary->cols,binary->rows));

                //add the point to the list
                mineralPoints.push_back(thisPoint);


                //NOW WE WILL ADD THIS POINT TO THE CRATER IMAGE
                //convert this point to be on the image
                Point imagePoint = pointToImage(thisPoint);

                //just a sanity check to make sure the point is on the image
                if(imagePoint.x < 0 || imagePoint.y < 0){continue;}

                //set the pixel to be 255, white
                craterImage.at<uchar>(imagePoint.y,imagePoint.x) = 255;
            }
        }
    }
    Mat craterImageDebug;
    cvtColor(craterImage, craterImageDebug,CV_GRAY2RGB);
    resize(craterImageDebug,craterImageDebug,Size(),3,3);
    craterImageDebug.copyTo((*drawingImage)(Rect(0,0,craterImageDebug.cols,craterImageDebug.rows)));
}



/**
 * This method uses the mineralPoints image and finds the best attack plan.
 * It returns a point of where to put the collector down
 */
CollectingLocation* MineralScorer::findBestPath() {
    //Now we are going to go through the image with all the minerals as white spots and look
    //for the biggest black spot.
    int stepSizeX = craterImage.cols/25;
    int stepSizeY = stepSizeX;


    //we need these to figure out if we are within or outside the crater
    double craterSlope = (CRATER1_BOTTOM_RIGHT_Y - CRATER1_TOP_LEFT_Y)/
            (CRATER1_BOTTOM_RIGHT_X-CRATER1_TOP_LEFT_X);
    double craterIntercept = CRATER1_TOP_LEFT_Y;



    //the best CollectingLocation (this includes a public score member variable)
    CollectingLocation* bestCollectingLocation = new CollectingLocation();
    //this will iterate through the image and score all the locations and pick the best
    for(int y = 0; y < craterImage.rows; y += stepSizeY){
        for(int x = 0; x < craterImage.cols; x += stepSizeX){
            //this will store all the data about this point
            CollectingLocation* thisLocation = new CollectingLocation();
            //Get the real coordinate of this point
            thisLocation->setDropZone(imagePointToWorld(Point(x,y)));
            //set the collecting location to be the same as the drop zone by default
            thisLocation->setCollectingLocation(imagePointToWorld(Point(x,y)));
            //continue if we are outside the crater
            if(thisLocation->dropZone.y > craterSlope * thisLocation->dropZone.x + craterIntercept){ continue; }

            //score this collecting location (this will modify thisLocation's data)
            scoreImageLocation(thisLocation);

            //if we have set a new record, record it
            if(thisLocation->score > bestCollectingLocation->score){
                bestCollectingLocation = thisLocation;
            }
        }
    }
    Point dropZone = pointToImage(bestCollectingLocation->dropZone);
    circle(*drawingImage,Point(dropZone.x, dropZone.y)*3,4,Scalar(0,255,255),-1);

    Point mineralLocation = pointToImage(bestCollectingLocation->collectingLocation);
    circle(*drawingImage,Point(mineralLocation.x, mineralLocation.y)*3,4,Scalar(255,255,0),-1);


    Point topLeft = pointToImage(Point2d(CRATER1_TOP_LEFT_X,CRATER1_TOP_LEFT_Y));
    Point bottomRight = pointToImage(Point2d(CRATER1_BOTTOM_RIGHT_X,CRATER1_BOTTOM_RIGHT_Y));
    line(*drawingImage,topLeft*3,bottomRight*3,Scalar(255,0,0),1);

    //return the location of the circle with the biggest radius
    return bestCollectingLocation;
}




/**
 * This method scores a location on the crater image by modifying the score member
 * variable of the collectingLocation
 * @param collectingLocation the crater image location of the point
 */
void MineralScorer::scoreImageLocation(CollectingLocation* collectingLocation) {

    //this is the score of the point we are looking at
    double pointScore = 0;


    ///THE FIRST STEP IS TO EXTEND A CIRCLE UNTIL IT HITS SOMETHING, THE BIGGER THE BETTER SCORE////
    int radiusCircle = 1;
    int radiusStepSize = 1;
    //this loop will grow a circle until it hit's anything to see how big of a drop zone we have
    while(true){
        //this is a count of the white (mineral) pixels in the circle
        int numWhitePixelsInCircle = 0;
        for(int relativeY = -radiusCircle; relativeY < radiusCircle; relativeY ++){
            for(int relativeX = -radiusCircle; relativeX < radiusCircle; relativeX++){
                //get the image location
                int xImage = collectingLocation->dropZoneScreen.x + relativeX;
                int yImage = collectingLocation->dropZoneScreen.y + relativeY;
                //continue if we are not within the circle
                if(pow(relativeX,2) + pow(relativeY,2) > pow(radiusCircle,2)){ continue; }

                //now we will treat it black if it is outside the bounds of the image
                if(xImage < 0 || yImage < 0 ||
                   xImage >= craterImage.cols ||
                   yImage >= craterImage.rows){
                    numWhitePixelsInCircle ++;
                    continue;
                }

                //if this is a white pixel, increment the counter
                if(craterImage.at<uchar>(yImage,xImage) != 0){
                    numWhitePixelsInCircle ++;
                }
            }
        }
        //break if there are a few white pixels
        if(numWhitePixelsInCircle > 0){break;}

        radiusCircle += radiusStepSize;//increment the radius

        if(radiusCircle > craterImage.cols/3){break;}//sanity check
    }


    //you can see that there are problems with this algorithm if the CRATER_IMAGE_SIZE_X
    //doesn't equal the CRATER_IMAGE_SIZE_Y since the circle is in image space
    double radiusCircleWorldScale = ((double) radiusCircle/craterImage.cols) * CRATER_IMAGE_SIZE_X;

    //we will give this radius a full score if it is equal to or greater than 15 cm
    double radiusScore = radiusCircleWorldScale/10.0;
    if(radiusScore > 1){radiusScore = 1;}//don't allow scores above 1

    if(radiusCircleWorldScale > 3){
        pointScore += radiusScore * 1.5;
    }else{
        pointScore -= 30;
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////

















    //////////////The next part of the score is if there are minerals ahead////////////////////////
    //we are going to ray trace around this area so we need to know the angle at which to do so
    //to know the angle we need to know where the robot is and we will assume we collect
    //straight
    double craterYIntercept = CRATER1_TOP_LEFT_Y + 30;//add some extra to make this intersect earlier
    //now we calculate the crater slope
    double craterSlope = (CRATER1_BOTTOM_RIGHT_Y - CRATER1_TOP_LEFT_Y)/
                         (CRATER1_BOTTOM_RIGHT_X-CRATER1_TOP_LEFT_X);

    //now to get the robot collecting location, we will extend a line from the robot and at
    //the robot angle and intersect it with the line that is the enlarged crater
    Point2d* robotCollectingLocation = lineIntersection(Point2d(0,craterYIntercept),craterSlope,
            Point2d(robotX,robotY),tan(robotAngle));

    circle(*drawingImage,pointToImage(*robotCollectingLocation)*3,3,Scalar(0,255,0),-1);



    //now let's calculate the angle from the robot to this point. We can assume we will
    //line up at this angle
    double angleFromRobotToThisPoint = atan2(collectingLocation->dropZone.y-robotCollectingLocation->y,
                                             collectingLocation->dropZone.x-robotCollectingLocation->x);
    //we will ray trace opposite this angle
    double angleRayTracing = angleWrap(angleFromRobotToThisPoint);


    //this is the maximum amount forwards we will ray trace, if there are cubes farther
    //away from this point than the max, they won't count towards our score
    double maxRayTraceDistance = 30;

    //if the ray doesn't hit anything this is the maximum number of ray tracing iterations
    int rayTracingMaxIters = 15;

    //this is the distance away from this point where we hit a mineral
    double firstHitDistanceAway = -1;

    //Now start at a distance of 0, and iterate until we hit the max distance or
    //(hopefully) we hit something
    for(double distance = 0; distance < maxRayTraceDistance; distance += maxRayTraceDistance/rayTracingMaxIters){
        Point2d currRealCoordinate = Point2d();
        //calculate where we currently are
        currRealCoordinate.x = (cos(angleRayTracing) * distance) + collectingLocation->dropZone.x;
        currRealCoordinate.y = (sin(angleRayTracing) * distance) + collectingLocation->dropZone.y;

        //convert the real coordinate to a point on the bitmap
        Point screenCoordinate = pointToImage(currRealCoordinate);

        //if we ever reach outside the image, break since it would be out of bounds
        if(screenCoordinate.x < 0 || screenCoordinate.y < 0){break;}

        //now we can see if we have hit a mineral by looking at this point in the crater image
        bool isMineral = craterImage.at<uchar>(screenCoordinate.y,screenCoordinate.x) != 0;


        //if this is a mineral, record the distance we are away and break
        if(isMineral){
            firstHitDistanceAway = distance;
            line(*drawingImage,collectingLocation->dropZoneScreen * 3, screenCoordinate*3,Scalar(255,0,0));
            break;
        }
    }

//    circle(*drawingImage,point * 3,3,Scalar(255,0,0));


    //if we actually collided with something we can up our score
    if(firstHitDistanceAway >= 0){
        //we don't need to clip this since this is always below 1
        double hitDistanceScore = firstHitDistanceAway/maxRayTraceDistance;
        pointScore += (1-hitDistanceScore)*1.5;//maybe add a weight
    }else{
        pointScore -= 30;
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////




    //The next component of the score will be the distance to where the robot would be
    //collecting (closer is better)

    double distanceAway = hypot(robotCollectingLocation->x-collectingLocation->dropZone.x,
            robotCollectingLocation->y-collectingLocation->dropZone.y);
    double distancePercent = distanceAway/130.0;
    if(distancePercent > 1){ distancePercent = 1; }

    //score is inversely proportional to distance away
    pointScore += (1 - distancePercent) * 3;



    //next we don't want the angle this point makes with the robot to be large
    double anglePercent = abs(angleWrap(angleRayTracing-(-0.75*PI))) / (40 * PI/180.0);
    if(anglePercent > 1){anglePercent = 1;}
    pointScore -= anglePercent * 0.6;
    if(anglePercent > 0.9){
        pointScore -= 30;
    }



    //kill it if it's really close to the corner
    double distanceToCorner = hypot(collectingLocation->dropZone.x,collectingLocation->dropZone.y);
    if(distanceToCorner < 30){
        pointScore -= 30;
    }





    //now we are going to make sure it isn't too far to the crater

    Point2d* craterIntersection = lineIntersection(Point2d(0,CRATER1_TOP_LEFT_Y),craterSlope,
                                                   Point2d(collectingLocation->dropZone.x,collectingLocation->dropZone.y),
                                                   tan(-0.75*PI));

    if(hypot(collectingLocation->dropZone.x-craterIntersection->x,collectingLocation->dropZone.y-craterIntersection->y) < 20){
        pointScore -= 0.5;
    }


    collectingLocation->setScore(pointScore);

}






/**
 * Finds the intersection of a line segment and a circle
 * @param circleX the x position of the circle
 * @param circleY the y position of the circle
 * @param r: radius of the circle
 * @param lineX1 start x of the line
 * @param lineY1 start y of the line
 * @param lineX2 end x of the line
 * @param lineY2 end y of the line
 * @return a vector of Point2d that are the intersection
 */
vector<Point2d>* MineralScorer::lineCircleIntersection(double circleX, double circleY, double r,
                                                      double lineX1, double lineY1,
                                                      double lineX2, double lineY2){
    //make sure the points don't exactly line up so the slopes work
    if(abs(lineY1- lineY2) < 0.003){
        lineY1 = lineY2 + 0.003;
    }
    if(abs(lineX1- lineX2) < 0.003){
        lineX1 = lineX2 + 0.003;
    }

    //calculate the slope of the line
    double m1 = (lineY2 - lineY1)/(lineX2-lineX1);

    //the first coefficient in the quadratic
    double quadraticA = 1.0 + pow(m1,2);

    //shift one of the line's points so it is relative to the circle
    double x1 = lineX1-circleX;
    double y1 = lineY1-circleY;


    //the second coefficient in the quadratic
    double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1,2) * x1);

    //the third coefficient in the quadratic
    double quadraticC = ((pow(m1,2)*pow(x1,2)) - (2.0*y1*m1*x1) + pow(y1,2)-pow(r,2));


    vector<Point2d>* allPoints = new vector<Point2d>();



    //this may give an error so we use a try catch
    try{
        //now solve the quadratic equation given the coefficients
        double xRoot1 = (-quadraticB + sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);

        //we know the line equation so plug into that to get root
        double yRoot1 = m1 * (xRoot1 - x1) + y1;


        //now we can add back in translations
        xRoot1 += circleX;
        yRoot1 += circleY;

        //make sure it was within range of the segment
        double minX = lineX1 < lineX2 ? lineX1 : lineX2;
        double maxX = lineX1 > lineX2 ? lineX1 : lineX2;
        if(xRoot1 > minX && xRoot1 < maxX){
            allPoints->push_back(Point2d(xRoot1,yRoot1));
        }

        //do the same for the other root
        double xRoot2 = (-quadraticB - sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);

        double yRoot2 = m1 * (xRoot2 - x1) + y1;
        //now we can add back in translations
        xRoot2 += circleX;
        yRoot2 += circleY;

        //make sure it was within range of the segment
        if(xRoot2 > minX && xRoot2 < maxX){
            allPoints->push_back(Point2d(xRoot2,yRoot2));
        }
    }catch(Exception e){
        //if there are no roots
    }
    return allPoints;
}




/**
 * Takes intersection of two lines defined by one point and their slopes
 */
Point2d* MineralScorer::lineIntersection(Point2d point1, double m1, Point2d point2, double m2) {

    //solves for the x pos of the intercept
    double xIntercept = ((-m2 * point2.x) + point2.y + (m1 * point1.x) - point1.y)/(m1-m2);
    //plug into any equation to get y
    double yIntercept = m1 * (xIntercept - point1.x) + point1.y;
    //return the intersection point
    return new Point2d(xIntercept,yIntercept);
}



/**
 * Makes sure an angle is in the range -180 to 180
 * @param angle
 * @return
 */
double MineralScorer::angleWrap(double angle) {
    double pi = 3.14159265358979;
    while (angle<-pi){
        angle += 2.0*pi;
    }
    while (angle>pi){
        angle -= 2.0*pi;
    }
    return angle;
}

/**
 * Converts a real point to a point on the craterImage if possible
 * @param point the real coordinate
 * @return the coordinate on the image
 */
Point MineralScorer::pointToImage(Point2d point) {
    double xPercentCrater = point.x/CRATER_IMAGE_SIZE_X;
    double yPercentCrater = point.y/CRATER_IMAGE_SIZE_Y;

    //Stop if we are outside the image
    if(xPercentCrater < 0 || yPercentCrater < 0
       || xPercentCrater >= 1 || yPercentCrater >= 1) {
        return Point(-1,-1);
    }


    //now that we have made sure this point is on the image,
    //calculate where on the image it is
    int xImage = (int) (xPercentCrater*RESOLUTION_X);
    int yImage = (int) (yPercentCrater*RESOLUTION_Y);

    return Point(xImage,yImage);
}



/**
 * This will convert a point on the image matrix to a real life coordinate
 * @return a Point2d object that is in world coordinates
 */
Point2d MineralScorer::imagePointToWorld(Point point) {
    return cv::Point2d(((double) point.x/craterImage.cols)* CRATER_IMAGE_SIZE_X,
                       ((double) point.y/craterImage.rows)* CRATER_IMAGE_SIZE_Y);
}


