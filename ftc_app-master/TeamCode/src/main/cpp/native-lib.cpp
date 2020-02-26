#include <jni.h>
#include <string>
#include "opencv2/core.hpp"
#include "Telemetry.h"
#include <vector>
#include <cmath>
#include <sstream>
#include "opencv2/opencv.hpp"



Telemetry* m_Telemetry;


using namespace cv;
using namespace std;

extern "C"
JNIEXPORT jstring JNICALL
Java_RobotUtilities_TelemetryAdvanced_getLine(JNIEnv *env, jobject instance, jint line) {


    return  env->NewStringUTF(m_Telemetry->getString((int) line)->c_str());
}


extern "C"
JNIEXPORT void JNICALL
Java_RobotUtilities_TelemetryAdvanced_init(JNIEnv *env, jobject instance, jint size_x, jint size_y) {
    m_Telemetry = new Telemetry(size_x, size_y);
}


extern "C"
JNIEXPORT void JNICALL
Java_RobotUtilities_TelemetryAdvanced_putChar(JNIEnv *env, jobject instance, jint x, jint y,
                                              jint r, jint g, jint b, jchar myChar) {
    m_Telemetry->putChar(x,y,r,g,b, myChar,true);
}

extern "C"
JNIEXPORT void JNICALL
Java_RobotUtilities_TelemetryAdvanced_clear(JNIEnv *env, jobject instance) {
    m_Telemetry->clear();
}

extern "C"
JNIEXPORT void JNICALL
Java_RobotUtilities_TelemetryAdvanced_drawLine(JNIEnv *env, jobject instance, jdouble x1, jdouble y1,
                                               jdouble x2, jdouble y2, jint r, jint g, jint b, jchar myChar) {
    x1 /= 358.0;
    y1 /= 358.0;
    x2 /= 358.0;
    y2 /= 358.0;

    m_Telemetry->drawLineScaled(x1,y1,x2,y2,r,g,b, myChar);
}

extern "C"
JNIEXPORT void JNICALL
Java_RobotUtilities_TelemetryAdvanced_drawField(JNIEnv *env, jobject instance, jdouble center_x,
                                                jdouble center_y, jdouble zoom) {
    m_Telemetry->drawField(center_x, center_y,zoom);
}

extern "C"
JNIEXPORT void JNICALL
Java_RobotUtilities_TelemetryAdvanced_drawRobot(JNIEnv *env, jobject instance, jdouble robotX,
                                                jdouble robotY, jdouble robotAng) {

    robotX /= 358.0;
    robotY /= 358.0;
    m_Telemetry->drawRobot(robotX,robotY,robotAng);

}

extern "C"
JNIEXPORT void JNICALL
Java_RobotUtilities_TelemetryAdvanced_putCharField(JNIEnv *env, jobject instance, jdouble x,
                                                   jdouble y, jchar myChar) {

    x /= 358;
    y /= 358;

    //to do this we just draw a really small line :0
    m_Telemetry->drawLineScaled(x,y,x+0.001,y+0.001,255,255,255,myChar);

}


extern "C"
JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_robotcontroller_Vision_Vision_firstPerson(JNIEnv *env, jclass type,
                                                                        jlong addrRgba) {
    Mat& rgba = *(Mat*) addrRgba;
    m_Telemetry->drawFPV(&rgba);
}






extern "C"
JNIEXPORT jint JNICALL
Java_org_firstinspires_ftc_robotcontroller_Vision_Vision_readBallPattern(JNIEnv *env, jclass type,
                                                                    jlong addrRgba,jdouble sample1X,
                                                                         jdouble sample2X, jdouble sample3X,
                                                                         jdouble sample1Y,jdouble sample2Y,
                                                                         jdouble sample3Y) {

    //get our img by converting the long into a pointer
    Mat& img = *(Mat*) addrRgba;
//    flip(img,img,0);

//    Point2f src_center(img.cols/2.0F, img.rows/2.0F);
//    Mat rot_mat = getRotationMatrix2D(src_center, -90, 1.0);
//    Mat dst;
//    warpAffine(img, img, rot_mat, img.size());

    //amount to downsize image. scaleSize of 4 = 1/16 the size
    const double scaleSize = 8;


    //let's not care about the alpha channel
    cvtColor(img,img,COLOR_RGBA2RGB);


    //this is what we will use for processing stuff (at lower resolution)
    Mat processingImage;

    //rescale the main image into the processing image. scaleSize of 4 = 1/16 the size
    resize(img, processingImage, Size(), 1/scaleSize, 1/scaleSize, INTER_CUBIC);
    //blur the image to filter out the noise
    blur(processingImage,processingImage,Size(3,3));


    //holds an hsv version of the processing image
    Mat hsv;
    //convert to hsv, store in hsv mat
    cv::cvtColor(processingImage,hsv,COLOR_RGB2HSV);




    double minHue = 25.0;
    double maxHue = 30.0;




    int hSum = 0;
    int sSum = 0;
    int vSum = 0;
    int pixelCount = 0;


    /* locations of the sample, we are sampleing from the mat hsv,
     * which is converted from processing */
    double sampleRadius = 40;
    int sampleX_start = (int) ((sample1X-sampleRadius) / scaleSize);
    int sampleX_end = (int) ((sample1X+sampleRadius)/ scaleSize);
    int sampleY_start = (int) ((sample1Y-sampleRadius) / scaleSize);
    int sampleY_end = (int) ((sample1Y +sampleRadius) / scaleSize);



    for(int y = sampleY_start; y < sampleY_end; y ++){
        for(int x = sampleX_start; x < sampleX_end; x ++){
            hSum += hsv.at<Vec3b>(y,x)[0];
            sSum += hsv.at<Vec3b>(y,x)[1];
            vSum += hsv.at<Vec3b>(y,x)[2];
            pixelCount ++;
        }
    }
    //convert the hAverage back to degrees so we can use it
    double hAverage1 = (double) hSum/pixelCount;
    double sAverage1 = (double) sSum/pixelCount;
    double vAverage1 = (double) vSum/pixelCount;
    //draws where the sample was taken
    rectangle(img,Point(sampleX_start * (int) scaleSize,sampleY_start * (int) scaleSize),
              Point(sampleX_end * (int) scaleSize,sampleY_end * (int) scaleSize),
              Scalar(255,255,0),3);








    sampleX_start = (int) ((sample2X-sampleRadius) / scaleSize);
    sampleX_end = (int) ((sample2X+sampleRadius)/ scaleSize);
    sampleY_start = (int) ((sample2Y-sampleRadius) / scaleSize);
    sampleY_end = (int) ((sample2Y +sampleRadius) / scaleSize);
    hSum = 0;
    sSum = 0;
    vSum = 0;
    pixelCount = 0;
    for(int y = sampleY_start; y < sampleY_end; y ++){
        for(int x = sampleX_start; x < sampleX_end; x ++){
            hSum += hsv.at<Vec3b>(y,x)[0];
            sSum += hsv.at<Vec3b>(y,x)[1];
            vSum += hsv.at<Vec3b>(y,x)[2];
            pixelCount ++;
        }
    }
    //convert the hAverage back to degrees so we can use it
    double hAverage2 = (double) hSum/pixelCount;
    double sAverage2 = (double) sSum/pixelCount;
    double vAverage2 = (double) vSum/pixelCount;
    //draws where the sample was taken
    rectangle(img,Point(sampleX_start * (int) scaleSize,sampleY_start * (int) scaleSize),
              Point(sampleX_end * (int) scaleSize,sampleY_end * (int) scaleSize),
              Scalar(255,255,0),3);










    sampleX_start = (int) ((sample3X-sampleRadius) / scaleSize);
    sampleX_end = (int) ((sample3X+sampleRadius)/ scaleSize);
    sampleY_start = (int) ((sample3Y - sampleRadius) / scaleSize);
    sampleY_end = (int) ((sample3Y + sampleRadius) / scaleSize);

    hSum = 0;
    sSum = 0;
    vSum = 0;
    pixelCount = 0;
    for(int y = sampleY_start; y < sampleY_end; y ++){
        for(int x = sampleX_start; x < sampleX_end; x ++){
            hSum += hsv.at<Vec3b>(y,x)[0];
            sSum += hsv.at<Vec3b>(y,x)[1];
            vSum += hsv.at<Vec3b>(y,x)[2];
            pixelCount ++;
        }
    }
    //convert the hAverage back to degrees so we can use it
    double hAverage3 = (double) hSum/pixelCount;
    double sAverage3 = (double) sSum/pixelCount;
    double vAverage3 = (double) vSum/pixelCount;
    //draws where the sample was taken
    rectangle(img,Point(sampleX_start * (int) scaleSize,sampleY_start * (int) scaleSize),
              Point(sampleX_end * (int) scaleSize,sampleY_end * (int) scaleSize),
              Scalar(255,255,0),3);

    ///////////////////////////////////////////////////////////////////////////////////////////////



    double targetHue = 23;
    double targetV = 230;
    double targetS = 255;
    double error1 = abs(hAverage1 - targetHue) + abs(sAverage1 - targetS) + abs(vAverage1 - targetV);
    double error2 = abs(hAverage2 - targetHue) + abs(sAverage2 - targetS) + abs(vAverage2 - targetV);
    double error3 = abs(hAverage3 - targetHue) + abs(sAverage3 - targetS) + abs(vAverage3 - targetV);




    int whereIsCube;
    if(error1 <= 200 && error1 <= error2){
        circle(img,Point(sample1X,sample1Y),30,Scalar(255,255,0),-1);
        whereIsCube = 0;
    }else{
        if(error2 <= 200 && error2 <= error1 ){
            circle(img,Point(sample2X,sample2Y),30,Scalar(255,255,0),-1);
            whereIsCube = 1;
        }else{
            circle(img,Point(sample3X,sample3Y),30,Scalar(255,255,0),-1);
            whereIsCube = 2;
        }
    }



//    String display = "hAv: " + std::to_string((int) hAverage1) +
//            "sAv: " + std::to_string((int) sAverage1) +
//            "vAv: " + std::to_string((int) vAverage1);
    String display = "error1: " + std::to_string((int) error1) +
                     "error2: " + std::to_string((int) error2) +
                     "error3: " + std::to_string((int) error3);



    putText(img,display,Point(100,150),3,2,Scalar(255,0,0),2);
    putText(img,"sample1X: " + to_string(sample1X),Point(100,200),3,2,Scalar(255,0,0),2);





    Mat booleanImage;

    //Convert hsv to a boolean image based on if the color meets the hsv requirements or not
    inRange(hsv,Scalar(minHue,100,75),Scalar(maxHue,255,255),booleanImage);


    return whereIsCube;//return the cube



//    rectangle(img,Point(153,153),Point(197,197),Scalar(hAverage,sAverage,vAverage),-1);


//    cv::inRange(hsv,Scalar(172,100,20),Scalar(179,255,255),hsv2);
//    we need to do an or to apply equal margin on both sides
//    because red is 0 on the HSV color wheel
//    bitwise_or(hsv1,hsv2,booleanImage);




//
//    ////////////////////////////////////Detect Blobs//////////////////////////////
//    // Setup SimpleBlobDetector parameters.
//    SimpleBlobDetector::Params params;
//    params.filterByColor = true;
//    //this is a binary image, so only look at pixels with a 255 value.
//    params.blobColor = 255;
//    //Lets define a reasonable min and max area, since balls will never be giant, and will
//    //never be only 1 pixel big.
//    params.filterByArea = true;
//    params.minArea = 3*3;
//    params.maxArea =50*50;
//
//
//    params.filterByCircularity = false;
//    params.filterByConvexity = false;
//    params.filterByInertia = false;
//    //how much the blobs must look like circles -> we can have a high tollerance because
//    //we know the position of balls on the floor, so we will use their radius to confirm
//    //that they are at the predicted distance.
//
//
//    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
//    // SimpleBlobDetector::create creates a smart pointer.
//    // So you need to use arrow ( ->) instead of dot ( . )
//    std::vector<KeyPoint> keypoints;
//    detector->detect( booleanImage, keypoints);
//    //////////////////////////////////////////////////////////////////////////////
//
//    //scale the image back up after doing the blob detection
////    resize(drawingImage, drawingImage, Size(), scaleSize, scaleSize, INTER_CUBIC);
//
//
//
//
////    for(int y = 0; y < booleanImage.rows; y ++){
////        for(int x = 0; x < booleanImage.cols; x++){
////            rectangle(img,x*Size)
////        }
////    }
//
//
//
//    Mat boolImageScaled;//this will hold a scaled up version of booleanImage for debugging.
//    resize(booleanImage, boolImageScaled, Size(), scaleSize, scaleSize);
//
//    Mat blackAndGreen(img.rows,img.cols,CV_8UC3);
//
//    blackAndGreen.setTo(Scalar(0,255,0),boolImageScaled);
//    addWeighted(blackAndGreen,0.5,img,1.0,0.0,img);
//
//
//    //////////////////Overlay the Original Image for half of the screen///////////////////
//    //First, crop out what we want to overlay. It will be on the right half of the screen
//    //Top left: cols/2, 0 With: cols/2 Height: rows
////    Mat originalCropped = img(
////            Rect(originalCameraImage.cols/2 ,0,
////                 originalCameraImage.cols/2,originalCameraImage.rows));
////    //Now, copy what we have cropped onto the right half of the drawing image.
////    originalCropped.copyTo(drawingImage(
////            Rect(drawingImage.cols/2, 0,
////                 drawingImage.cols/2, drawingImage.rows)));
//    //////////////////////////////////////////////////////////////////////////////////////
//
//
//
//    double lowestY = 100000;
//    double lowestX = 100000;
//    double lowestRadius = 100000;
//
//    for(int i = 0; i<keypoints.size();i++){
//        //multiply all the values by scaleSize since we scaled the image down initially
//        keypoints[i].pt.x *= scaleSize;
//        keypoints[i].pt.y *= scaleSize;
//        keypoints[i].size *= scaleSize;
//
//
//        Point2f p = keypoints[i].pt;
//        float currentBallRadius = keypoints[i].size;
//
//        double thisRadius = (currentBallRadius/2.0f);
//        float thisX = p.x;
//        float thisY = p.y;
//
//        rectangle(img,Point(thisX - thisRadius,thisY - thisRadius),
//                  Point(thisX + thisRadius,thisY+thisRadius),
//                  Scalar(0,255,0,1),3);
//
//        if(p.y < lowestY){
//            lowestRadius =  thisRadius;
//            lowestY = thisY;
//            lowestX = thisX;
//        }
//    }
//    if(lowestY <  10000){
//        jfieldID fid = env->GetStaticFieldID(type,"bestBallX","D");
//        env->SetStaticDoubleField(type, fid, lowestX/img.cols);
//
//        jfieldID fid1 = env->GetStaticFieldID(type,"bestBallY","D");
//        env->SetStaticDoubleField(type, fid1, lowestY/img.rows);
//
//
//        rectangle(img,Point(lowestX - lowestRadius,lowestY - lowestRadius),
//                  Point(lowestX + lowestRadius,lowestY+lowestRadius),
//                  Scalar(0,255,0,1),3);
//    }



}