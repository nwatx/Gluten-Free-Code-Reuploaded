//
// PitScanner.cpp
// This class scans the pit for collecting options
// Created by peter on 2/24/2019.
//

#include "opencv2/core.hpp"
#include "RotatedAverageCalculator.h"
#include "MineralScorer.h"
#include <jni.h>
#include <opencv2/imgproc.hpp>

//need to use the cv and std namespace
using namespace cv;
using namespace std;

RotatedAverageCalculator* myRotatedAverageCalculator;
MineralScorer* myMineralScorer;

extern "C"
JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_robotcontroller_CompleteAutonomous_PitScanner_scanPit(JNIEnv *env, jclass type,
        jlong addrRgba, jdouble craterAngle,jdouble robotX, jdouble robotY, jdouble robotAngle,
        jdoubleArray jniXPositions, jdoubleArray jniYPositions, jboolean secondScan) {
    //amount to downsize image. scaleSize of 4 = 1/16 the size
    const double scaleSize = 4;

    //cast the jdouble to a double to get the crater angle on the screen
    double craterAngle_rad = (double) craterAngle;


    //get our image by converting the long into a pointer
    Mat &img = *(Mat *) addrRgba;

    //flip the image to the correct orientation
    flip(img,img,-1);
    //let's not care about the alpha channel
    cvtColor(img, img, COLOR_RGBA2RGB);

    //if we need to block the collector
    if((bool) secondScan){
        rectangle(img,Point(img.cols * (0.55-0.1), img.rows * 0.2),Point(img.cols*(0.55+0.1),img.rows),Scalar(0,255,0),-1);
        rectangle(img,Point(img.cols * (0.55-0.16),img.rows * 0.65),Point(img.cols * (0.55+0.16),img.rows),Scalar(0,255,0),-1);

        rectangle(img,Point(img.cols * (0.55-0.22),img.rows * 0.8),Point(img.cols * (0.55+0.22),img.rows),Scalar(0,255,0),-1);

    }

    //this is what we will use for processing stuff (at lower resolution)
    Mat processingImage;
    //rescale the main image into the processing image. scaleSize of 4 = 1/16 the size
    resize(img, processingImage, Size(), 1 / scaleSize, 1 / scaleSize, INTER_CUBIC);
    //blur the image to filter out the noise
    blur(processingImage, processingImage, Size(3, 3));


    //holds an hsv version of the processing image
    Mat hsv;
    //convert to hsv, store in hsv mat
    cv::cvtColor(processingImage, hsv, COLOR_RGB2HSV);





    ///////////////////////////////CRATER DETECTION//////////////////////////////////////
//TODO: USE THE CRATER TO ALIGN THE REST OF THE VISION
    //we'll need this to do rotated row averages
    myRotatedAverageCalculator = new RotatedAverageCalculator();

    //start the best error at a really high value
    double bestError = 9999999;
    int craterStart = 0;//this is the column that the crater starts on



    int lookAhead = processingImage.rows/25;
    //start at the bottom, move towards the top
    for(int i = processingImage.rows-1-lookAhead; i >= lookAhead * 2 && i > processingImage.rows * 0.2; i --){
        int left = (processingImage.cols/2) - 10;
        int right = (processingImage.cols/2) + 10;
        //get all the Vec3b rotated line averages from the four sample locations
        Vec3b black1 = *myRotatedAverageCalculator->
                getLineAverage(&hsv,i,craterAngle_rad,left,right,false,nullptr);
        Vec3b black2 = *myRotatedAverageCalculator->
                getLineAverage(&hsv,i+lookAhead,craterAngle_rad,left,right,false,nullptr);
        Vec3b grey1 = *myRotatedAverageCalculator->
                getLineAverage(&hsv,i-lookAhead*2,craterAngle_rad,left,right,false,nullptr);
        Vec3b grey2 = *myRotatedAverageCalculator->
                getLineAverage(&hsv,i-lookAhead*3,craterAngle_rad,left,right,false,nullptr);

        if(i == processingImage.rows-1-lookAhead){
            myRotatedAverageCalculator->getLineAverage(&hsv,i,craterAngle_rad,left,right,true,&img);
        }
        //the more v, the more black error
        double blackError = (black1[2] + black2[2])/(255.0 * 2.0);
        //the more difference from 255 on the pixel above, the more grey error
        double greyError = (abs(255-grey1[2]) + abs(255-grey2[2]))/(255.0*2.0);

        //sum up the total error of this crater position
        double totalError = blackError*2.0 + greyError*1.0 + ((double)(processingImage.rows - i)/processingImage.rows) * 0.2;
        if(totalError < bestError){
            bestError = totalError;
            craterStart = i;
        }

        circle(img,Point(30, (int) (i * scaleSize)),10,Scalar(black2[0],black2[1],black2[2]),-1);
    }

    double lineLeftY = ((0-(img.cols/2)) * tan(craterAngle_rad)) + (craterStart * scaleSize);
    double lineRightY = ((img.cols-(img.cols/2)) * tan(craterAngle_rad)) + (craterStart * scaleSize);
    line(img,Point(0,(int) lineLeftY),Point(img.cols,(int) lineRightY),Scalar(255,0,0),3);

    /////////////////////////////////////////////////////////////////////////////////////////////////
















    double cubeHue = 23;
    double cubeS = 230;
    double cubeV = 255;

    Mat threshCubes;
    inRange(hsv,Scalar(cubeHue-10,cubeS-75,150),Scalar(cubeHue+10,255,255),threshCubes);



    Mat threshBalls;
    inRange(hsv,Scalar(0,0,210),Scalar(255,50,255),threshBalls);



    Mat threshMinerals;//holds the or of the two thresholds
    bitwise_or(threshCubes,threshBalls,threshMinerals);//or them


    Mat threshMineralsScaled;//this will hold a scaled up version of threshMinerals for debugging.
    resize(threshMinerals, threshMineralsScaled, Size(), scaleSize, scaleSize);//scale it up by size

    Mat greenWhereCube(img.rows,img.cols,CV_8UC3);
    greenWhereCube.setTo(Scalar(0,255,0),threshMineralsScaled);//black where not in threshold, green where is
    addWeighted(greenWhereCube,0.2,img,1.0,0.0,img);




    //////////////////////////IT IS TIME TO SCORE THE MINERALS////////////////////////
    myMineralScorer = new MineralScorer();
    myMineralScorer->drawingImage = &img;
    myMineralScorer->updateRobotPosition((double) robotX, (double) robotY, (double) robotAngle);
    myMineralScorer->updateMinerals(&threshMinerals);

    CollectingLocation* dropZone = myMineralScorer->findBestPath();
    //////////////////////////////////////////////////////////////////////////////////









    /////////////////////////NOW WE NEED TO RETURN THE ARRAYS//////////////////////////////
    jboolean isCopyX;
    jdouble *xPositionsLink = env->GetDoubleArrayElements(jniXPositions,&isCopyX);
    jboolean isCopyY;
    jdouble *yPositionsLink = env->GetDoubleArrayElements(jniYPositions,&isCopyY);



    for(int i = 0; i < myMineralScorer->mineralPoints.size(); i ++){
//        string log = "Adding Point: (" + to_string(myMineralScorer->mineralPoints.at(i).x)
//                     + " , " + to_string( myMineralScorer->mineralPoints.at(i).y) + ")";
//
//        __android_log_write(ANDROID_LOG_ERROR,"ERROR_LOG", log.c_str());

        xPositionsLink[i] = (jdouble) myMineralScorer->mineralPoints.at(i).x;
        yPositionsLink[i] = (jdouble) myMineralScorer->mineralPoints.at(i).y;
    }
    //now we need to release the elements
    env->ReleaseDoubleArrayElements(jniXPositions,xPositionsLink,0);
    env->ReleaseDoubleArrayElements(jniYPositions,yPositionsLink,0);

    ////////////////////////////////////////////////////////////////////////////////////////


    //This divides the screen into segments so you can calibrate the y displacement
    for(int i = 0; i < 20; i ++){
        putText(img,to_string(i),Point((int) (img.cols * (0.5-0.1)),(int) (img.rows * (i/20.0))),1,1,Scalar(255,150,0));
        line(img,Point((int) (img.cols * (0.5-0.02)),(int) (img.rows * (i/20.0))),
             Point((int) (img.cols * (0.5+0.02)),(int) (img.rows * (i/20.0))),Scalar(255,0,0),1);
    }


    //these are used if you want to calibrate screen width as a function of y position
    line(img,Point((int) (img.cols * (0.5+0.1)),img.rows),
         Point((int) (img.cols * (0.5+0.1)),0),Scalar(255,0,0),1);
    line(img,Point((int) (img.cols * (0.5-0.1)),img.rows),
         Point((int) (img.cols * (0.5-0.1)),0),Scalar(255,0,0),1);


    ///////////////////////NOW WE NEED TO RETURN THE BEST LOCATION//////////////////////////
    jfieldID fid = env->GetStaticFieldID(type,"bestDropX","D");
    env->SetStaticDoubleField(type, fid, dropZone->dropZone.x);
    jfieldID fid2 = env->GetStaticFieldID(type,"bestDropY","D");
    env->SetStaticDoubleField(type, fid2, dropZone->dropZone.y);


    jfieldID fid3 = env->GetStaticFieldID(type,"bestCubeX","D");
    env->SetStaticDoubleField(type, fid3, dropZone->collectingLocation.x);
    jfieldID fid4 = env->GetStaticFieldID(type,"bestCubeX","D");
    env->SetStaticDoubleField(type, fid4, dropZone->collectingLocation.y);
    ////////////////////////////////////////////////////////////////////////////////////////

    /*




    //all the starting ys of the stretches
    vector<int> startingYs = vector<int>();
    //all the ending ys of the stretches
    vector<int> endingYs = vector<int>();




    //this is the left edge of the crater on the image
    int craterLeftX = 0;
    //the right edge of the crater on the image
    int craterRightX = threshMinerals.cols-1;

    //the y position of the bottom of the crater
    int craterBottomY = craterStart;
    //this is the y position of the top of the crater
    int craterTopY = 0;

    //this is how much we increment x every time
    //we will do a step size of 1 tenth the crater size
    int stepSizeX = 1;//(craterRightX-craterLeftX)/10;

    //we will step by one pixel in the y
    int stepSizeY = 1;


    //start on the left edge of the crater, work towards the right
    for(int x = craterLeftX; x < craterRightX; x += stepSizeX){
        int longestStretchDistance = 0;//this is the distance of the longest line of nothing
        //start of that stretch
        int longestStartY = craterBottomY;
        //end of that stretch (below start)
        int longestEndY = craterTopY;

        //assume we start with a gap
        int currStartY = craterBottomY;

        //we are in a stretch
        bool inStretch =  threshMinerals.at<uchar>(craterBottomY,x) == 0;

        //start at the bottom edge, work to the top (y -= stepSizeY)
        for(int y = craterBottomY; y > craterTopY; y -= stepSizeY){

            //see if we are over a cube by if threshMinerals is black or not
            //anything not black (0) is a cube
            bool overCubeBottom = threshMinerals.at<uchar>(y,x) != 0;
            //now look one pixel above and see if there is a change
            bool overCubeNext = threshMinerals.at<uchar>(y - 1,x) != 0;

            //if we are over a cube and the next doesn't have anything this is the start of a stretch
            if((overCubeBottom != overCubeNext) ||
                (y <= craterTopY + stepSizeY && inStretch)){//if we are at the end, terminate the stretch
//                circle(img,Point((int) (x * scaleSize), (int) (y * scaleSize)),3,Scalar(255,255,0),-1);

                if(!inStretch){
                    currStartY = y;//this is the start of a stretch
                }else{
                    //get the stretch distance (the start of the stretch minus the end (y)
                    int stretchDistance = currStartY - y;
                    if(stretchDistance > longestStretchDistance){
                        longestStretchDistance = stretchDistance;
                        longestStartY = currStartY;
                        longestEndY = y;
                    }

//                    rectangle(img,Point((int) (x * scaleSize), (int) (y * scaleSize)),
//                              Point((int) (x * scaleSize), (int) (currStartY * scaleSize)),
//                              Scalar(255,0,0),1);

                    break;//normally we would keep going upwards but let's just break when we hit something
                }

                inStretch = !inStretch;//invert inStretch
            }
        }

        //save the details of the best stretch
        startingYs.push_back(longestStartY);
        endingYs.push_back(longestEndY);

        rectangle(img,Point((int) (x * scaleSize), (int) (longestEndY * scaleSize)),
                  Point((int) (x * scaleSize), (int) (longestStartY * scaleSize)),
                  Scalar(255,255,0),1);
    }






    //let's say we need four stretches wide to fit the collector
    int widthCollector = 17;

    //one of the priorities is that there are minerals above the stretch. However,
    //it should be capped at this amount of pixels and then they don't count anymore
    int maxMineralPixels = 20000000;

    //this is the maximum amount the algorithm is allowed to look forwards from the end of
    //the stretch. This means only minerals directly after the stretch will count
    double maxVerticalLookAhead = abs(craterTopY-craterBottomY)/5.0;

    //after the collecting area is this big, it won't matter that it is bigger
    int maxLengthCollector = processingImage.rows/2;

    //best scoring location
    int bestScore = -10000;
    //index of the stretch where the best scoring location is
    int bestIndex = 0;

    //start y of the best place
    int bestStartY = 0;
    //end y of the best place
    int bestEndY = 0;

    //Start on the left most edge where the collector can fit (widthCollector/2)
    //since we're looking at the middle
    for(int i = widthCollector/2; i < startingYs.size()-(widthCollector/2); i ++){
        int totalScore = 0;
        int thisEndingY = 0;//y value of the highest stretch ending y since this limits it
        int thisStartingY = 10000000;//y value of the lowest stretch ending y

        ///Mark the most conservative starting y first
        for(int stretch = i - (widthCollector/2); stretch < i + (widthCollector/2); stretch ++){
            //record the min and max if we define them
            if(startingYs.at(stretch) < thisStartingY){thisStartingY = startingYs.at(stretch);}
        }
        ///Next mark the most conservative ending y (has to be above starting y though)
        for(int stretch = i - (widthCollector/2); stretch < i + (widthCollector/2); stretch ++){
            if(endingYs.at(stretch) > thisEndingY && endingYs.at(stretch) < thisStartingY){thisEndingY = endingYs.at(stretch);}
        }



        int thisRange = abs(thisStartingY-thisEndingY);
        //clip thisRange so that it doesn't impact too much for crazy ranges
        if(thisRange > maxLengthCollector){ thisRange = maxLengthCollector; }



        double thisLengthPercent = ((double) thisRange/processingImage.rows);
        if(thisLengthPercent < 0.1){
            totalScore -= 10000;
        }
//        totalScore += (addToScore + addToScore2)*30.0;



        //convert the index to an x coordinate on the processing image (low res one)
        int xOnProcessingImage = i * stepSizeX;
//        circle(img, Point(xOnProcessingImage * scaleSize, 50 * scaleSize),3,Scalar(255,0,0),-1);

        double mineralScore = 0;//the number of pixels above the end of this location that have minerals

        ///Now tally up all the pixels that have minerals above this stretch
        for(int stretch = i - (widthCollector/2); stretch < i + (widthCollector/2); stretch ++){
            double x = stretch * stepSizeX;


            //start a bit above (less y) of the top of this location
            for(int y = thisEndingY; y >= craterTopY && y >= thisEndingY - maxVerticalLookAhead; y--){
                //now if there is a mineral pixel here, we will score it
                if(threshMinerals.at<uchar>(y,x) != 0){
                    //score decreases with increasing distance from the top
                    double verticalScore = 1.0 - ((double) abs(y-craterBottomY)/((abs(craterBottomY-craterTopY)))) * 1.0;

                    //this is used so that minerals not to the left or right of us score better
                    double horizontalScore = (1.0-(abs(stretch - i)/((double) widthCollector/2.0))) * 0.0;
                    //don't allow hurting your score
                    if(horizontalScore < 0){horizontalScore = 0;}

                    //get how many pixels we are away from the horizontal center of the image
                    double deltaPixelsFromCenter = abs(x - processingImage.cols/2.0);
                    //divide that by half the width so when it is at the edges, this is one
                    double horizontalPercent = deltaPixelsFromCenter / (processingImage.cols/2.0);
                    //1 minus that is the horizontal score (1 in the center)
                    double absoluteHorizontalScore = 1.0 - horizontalPercent;
                    //kill it if it's 80% turning
                    if(horizontalPercent > 0.8){
                        absoluteHorizontalScore = -1;
                    }

                    //increment by this score
                    mineralScore += verticalScore + horizontalScore + absoluteHorizontalScore * 0.5;

                    circle(img, Point(x * scaleSize, y * scaleSize),3,
                           Scalar(0,(int) (255.0 * (absoluteHorizontalScore+verticalScore)/2.0),0),-1);
                }
            }
        }

        if(mineralScore > maxMineralPixels){mineralScore = maxMineralPixels;}
        totalScore += mineralScore*10;//prioritize minerals (0 to 50)
        //make score worse if you are far away from the center (need to turn)
//        totalScore -= ((double) abs(xOnProcessingImage-(processingImage.cols/2))/(double) (processingImage.cols/2)) * 5; //(0 to 40)
//        totalScore -= ((double) abs(processingImage.rows-thisStartingY)/processingImage.rows) * 10000;//0 to 20

        //record everything if we beat the record
        if(totalScore > bestScore){
            bestScore = totalScore;
            bestIndex = i;
            bestStartY = thisStartingY;
            bestEndY = thisEndingY;
        }
    }


    //get the x positions on the high resolution image
    int xStart = (int) ((bestIndex-widthCollector/2) * scaleSize * stepSizeX);
    int xEnd = (int) ((bestIndex+widthCollector/2) * scaleSize * stepSizeX);

    rectangle(img,Point(xStart, bestStartY * scaleSize),
              Point(xEnd, bestEndY * scaleSize),
              Scalar(0,255,0),3);




    //these are percents
    double bestDropX = (bestIndex * stepSizeX)/(double) processingImage.cols;
    double bestDropY = bestEndY/(double) processingImage.rows;

    jfieldID fid = env->GetStaticFieldID(type,"bestDropX","D");
    env->SetStaticDoubleField(type, fid, bestDropX);
    jfieldID fid2 = env->GetStaticFieldID(type,"bestDropY","D");
    env->SetStaticDoubleField(type, fid2, bestDropY);


     */
}





