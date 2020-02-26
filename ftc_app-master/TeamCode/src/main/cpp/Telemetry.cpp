
#include "Telemetry.h"
#include "tel_char.h"
#include <cmath>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "ranked_tel_char.h"
#include <string>
#include <numeric>
#include <algorithm>
#include <functional>
#include <iostream>
#include <array>

using namespace cv;

int Telemetry::x_size = 0;
int Telemetry::y_size =0;



Telemetry::Telemetry(int size_x, int size_y) {
    allChars = vector<vector<tel_char>>();

    //populate our vector
    for(int y= 0; y < size_y; y ++){
        vector<tel_char> thisLine = vector<tel_char>();
        for(int x = 0; x < size_x; x ++){
            tel_char c = tel_char(0,0,0, ' ');
            thisLine.push_back(c);//initializing with spaces
        }
        allChars.push_back(thisLine);
    }
    x_size = size_x;
    y_size = size_y;
}

typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} hsv;

hsv rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}

rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
        case 0:
            out.r = in.v;
            out.g = t;
            out.b = p;
            break;
        case 1:
            out.r = q;
            out.g = in.v;
            out.b = p;
            break;
        case 2:
            out.r = p;
            out.g = in.v;
            out.b = t;
            break;

        case 3:
            out.r = p;
            out.g = q;
            out.b = in.v;
            break;
        case 4:
            out.r = t;
            out.g = p;
            out.b = in.v;
            break;
        case 5:
        default:
            out.r = in.v;
            out.g = p;
            out.b = q;
            break;
    }
    return out;
}

string * Telemetry::getString(int line) {




    if(allChars.size() == 0){
        return new string("not loaded");
    }

    vector<ranked_tel_char> ranked_tel_chars = vector<ranked_tel_char>();



//
//    //figure out how much the color changes for each of the chars in the row
//    for(int x = 1; x < allChars.at(line).size(); x ++){
//
//        tel_char* thisChar = &allChars.at(line).at(x);
//        tel_char* lastChar = &allChars.at(line).at(x-1);
//
//
//        rgb thisPixelRGB = {thisChar->r/255.0,thisChar->g/255.0,thisChar->b/255.0};
//        hsv thisPixelHSV = rgb2hsv(thisPixelRGB);
//
//        rgb lastPixelRGB = {lastChar->r/255.0,lastChar->g/255.0,lastChar->b/255.0};
//        hsv lastPixelHSV = rgb2hsv(lastPixelRGB);
//
//
//        double thisH = thisPixelHSV.h * 255;
//        double thisS = thisPixelHSV.s * 255;
//        double thisV = thisPixelHSV.v * 255;
//
//        double lastH = lastPixelHSV.h * 255;
//        double lastS = lastPixelHSV.s * 255;
//        double lastV = lastPixelHSV.v * 255;
//
//
//
//
//        int diffH = abs(thisH-lastH);
//        int diffS = abs(thisS-lastS);
//        int diffV = abs(thisV-lastV);
//
//
//
//        int sumDiff = (diffH * 2) + (diffS * 0.5) + (diffV * 0.5);
//
//
//        ranked_tel_chars.push_back(ranked_tel_char(thisChar,sumDiff));
//    }



    /*
    //now to rank the biggest color changes
    std::sort(ranked_tel_chars.begin(),ranked_tel_chars.end(),
              [](ranked_tel_char& a, ranked_tel_char& b) {
                  return a.colorDiffScore > b.colorDiffScore;
              });
              */

    //get 10 biggest color changes
//    for(int i = 0; i < ranked_tel_chars.size(); i ++){
//        tel_char * thisChar = ranked_tel_chars.at(i).myTelChar;
//        thisChar->isSignificant = i < 2;//you are significant if you place in top 10. UNLIKE ME :(
//    }



    string* thisLine = new string();



    tel_char* firstChar = &allChars.at(line).at(0);
    //string* hex = convToHex(firstChar->r,firstChar->g,firstChar->b);

    //thisLine->append("<font color='");
    //thisLine->append(*hex);
    //thisLine->append("'>");


    for(int x = 1; x < allChars.at(line).size(); x ++){
        tel_char* thisChar = &allChars.at(line).at(x);
        //string* hex = convToHex(thisChar->r,thisChar->g,thisChar->b);

        thisLine->push_back(thisChar->myChar);


//        if(thisChar->isSignificant){
//            //thisLine->append("</font><font color='");
//            //thisLine->append(*hex);
//            //thisLine->append("'>");
//            thisLine->push_back(thisChar->myChar);
//        }else{
//            thisLine->push_back(thisChar->myChar);
//        }
    }
    //thisLine->append("</font>");


    return thisLine;
}

void Telemetry::clear() {
    vector<tel_char> clear = vector<tel_char>();
    for(int x = 0; x < x_size; x ++){
        tel_char blank = tel_char(0,0,0, ' ');
        clear.push_back(blank);//go back to spaces
    }
    for(int i = 0; i < allChars.size(); i ++){
        allChars.at(i) = clear;
    }
}

void Telemetry::drawLine(int x1, int y1, int x2, int y2, int r, int g, int b, char myChar) {

    tel_char c = tel_char(r,g,b, myChar);

    //figure out our bounding box
    int x_start = x1 < x2 ? x1 : x2;
    int y_start = y1 < y2 ? y1 : y2;
    int x_end = x1 > x2 ? x1 : x2;
    int y_end = y1 > y2 ? y1 : y2;

    //calc our line's slope
    float lineSlope = x2 - x1 != 0 ?
                      (float) (y2-y1) / (float) (x2 - x1) //normal equation for slope
                                   : 5000000;//just make it big if x is same

    for(int y = y_start; y <= y_end; y++){
        for(int x = x_start; x <= x_end; x++){

            //stop it, get some help... if we outside bounds
            if(y >= allChars.size()
               or x >= allChars.at(0).size()
               or x < 0 or y < 0){continue;}



            int idealY = round(((float) (x-x1)* lineSlope)+y1);
            int idealX = round(((float) (y-y1)/lineSlope)+x1);

            //thickness
            if(abs(idealY - y) < 1 || abs(idealX-x) < 1 ){
                allChars.at(y).at(x) = c;
            }
        }
    }
}

void Telemetry::putChar(int x, int y, int r, int g, int b,char myChar,bool isSignificant) {
    //this will hold our pixel values
    tel_char c = tel_char(r,g,b,myChar);
    c.isSignificant = isSignificant;
    if(y >= allChars.size()
       or x >= allChars.at(0).size()
       or x < 0 or y < 0){return;}

    allChars.at(y).at(x) = c;
}

void Telemetry::drawField(double center_x, double center_y, double zoom) {
    this->zoom = zoom;
    this->center_x = center_x;
    this->center_y = center_y;

    //walls
    drawLineScaled(0,0, 1, 0,  255,0,0, 'o');
    drawLineScaled(1,1, 1, 0,  255,0,0, 'o');
    drawLineScaled(0,0, 0, 1,  255,0,0, 'o');
    drawLineScaled(0,1, 1, 1,  255,0,0, 'o');

    //horizontal lines
    drawLineScaled(0,0.5,0.25,0.5, 0,255,0,'o');
    drawLineScaled(0.75,0.5,1,0.5, 0,255,0,'o');

    //center diamond
    drawLineScaled(0.25,0.5,0.5,0.25, 0,255,0, 'o');
    drawLineScaled(0.25,0.5,0.5,0.75, 0,255,0, 'o');
    drawLineScaled(0.5,0.25,0.75,0.5, 0,255,0, 'o');
    drawLineScaled(0.5,0.75,0.75,0.5, 0,255,0, 'o');

    double fieldSize = 358.0;//length in cm of the field
    //craters
    drawLineScaled(0/fieldSize,120/fieldSize,48.0/fieldSize,100/fieldSize,0,255,0,'o');
    drawLineScaled(48.0/fieldSize,100/fieldSize,100/fieldSize,37/fieldSize,0,255,0,'o');
    drawLineScaled(100/fieldSize,37/fieldSize,110/fieldSize,0/fieldSize,0,255,0,'o');

    drawLineScaled(1,1.0-0.36,1.0-0.36,1,0,255,0,'o');

}

//this will use PERCENTAGES to draw on a field map with zoom and pan capabilities
void Telemetry::drawLineScaled(double x1, double y1, double x2, double y2,int r, int g, int b, char myChar) {
    y1 = 1-y1;
    y2 = 1-y2;

    x1 *= x_size;
    y1 *= x_size;
    x2 *= x_size;
    y2 *= x_size;


    //we want to zoom in and out in on the origin
    //center_x -= x_size/2;
    //center_y -= y_size/2;
    double rcenter_y = (-center_y*0.53) * x_size;
    double rcenter_x = (center_x * x_size);
    y1 = 0.53 * y1;
    y2 = 0.53 * y2;


    double real_y1 = ((y1-rcenter_y)* zoom);
    double real_y2 = ((y2-rcenter_y)* zoom);
    double real_x1 = (x1-rcenter_x) * zoom;
    double real_x2 = (x2-rcenter_x) * zoom;

    real_y1 += x_size /2;
    real_y2 += x_size /2;
    real_x1 += x_size /2;
    real_x2 += x_size /2;

    drawLine((int) real_x1,(int) real_y1,(int) real_x2,(int) real_y2, r,g,b, myChar);
}

void Telemetry::drawRobot(double robotX, double robotY, double robotAng) {
    const double halfC = M_PI / 180;//convert to rads by multiplying num by this
    robotAng -= 45 * halfC;
    double size = 0.08;
    for(int v = 0; v < 4; v++){
        double a1 = robotAng + (90 * halfC *v);
        double a2 = robotAng + (90 * halfC *(v+1));

        double x1 = (cos(a1)*size) + robotX;
        double y1 = (sin(a1)*size) + robotY;
        double x2 = (cos(a2)*size) + robotX;
        double y2 = (sin(a2)*size) + robotY;

        drawLineScaled(x1,y1,x2,y2,255,230,0, 'u');
    }
}

void Telemetry::drawFPV(Mat *displayMe) {
    if(displayMe == NULL){return;}
    if(displayMe->rows ==0){return;}
    if(y_size == 0 || x_size == 0){return;}

    Mat finalMat;
    displayMe->copyTo(finalMat);

    //transpose(finalMat, finalMat);
    //flip(finalMat, finalMat, +1);



//    Mat hsv;
//    cvtColor(finalMat,hsv,CV_RGB2HSV);
//
//
//
//    Mat thresh;
//    cv::inRange(hsv,Scalar(100,0,0,0),Scalar(135,255,255,255),thresh);
//
    //Mat finalMat(finalMat.rows,finalMat.cols,CV_8UC3,Scalar(0,0,0));
//
//
//    cv::blur(thresh,thresh,Size(10,10));

//    cv::inRange(thresh,Scalar(80),Scalar(255),thresh);

    //finalMat.copyTo(finalMat);//,thresh);


    int mat_x = finalMat.cols-1;
    int mat_y = finalMat.rows-1;



    for(int y = 0; y < y_size; y ++){
        for(int x = 0; x < x_size; x ++) {
            int actualY = (mat_y/y_size)*y;
            int actualX = (mat_x/x_size)*x;
            actualX *= 0.6666;
            actualX += mat_x/0.3;

            double r = finalMat.at<Vec4b>(actualY, actualX)[0];
            double g = finalMat.at<Vec4b>(actualY, actualX)[1];
            double b = finalMat.at<Vec4b>(actualY, actualX)[2];

            double intensity = (r-b)/255.0;//(r - b)/255.0;
//            intensity /=3.0;
            intensity *= 1.3;

            char c = ' ';
            if(intensity > 0.2){
                c = '.';
            }
            if(intensity > 0.4){
                c = '*';
            }
            if(intensity > 0.6){
                c = 'b';
            }
            if(intensity > 0.8){
                c = '@';
            }

            bool isSignificant = false;//we need to decide if this char matters for color or not because it is so expensive
            putChar(x,y,r,g,b,c,isSignificant);//this last parameter is if this char will get a unique color
        }
    }
}

string *Telemetry::convToHex(int r, int g, int b) {
    stringstream stream1;
    stream1 << std::hex << r;
    string red( stream1.str() );
    if(red.size() < 2){red = "0" + red;}

    stringstream stream2;
    stream2 << std::hex << g;
    string green( stream2.str() );
    if(green.size() < 2){green = "0" + green;}


    stringstream stream3;
    stream3 << std::hex << b;
    string blue( stream3.str() );
    if(blue.size() < 2){blue = "0" + blue;}



    string* hex = new string();
    hex->append("#");

    hex->append(red);
    hex->append(green);
    hex->append(blue);
    return hex;
}

void Telemetry::putCharField(double x, double y, int r, int g, int b, char myChar) {

    y = 1-y;

    x *= x_size;
    y *= x_size;


    //we want to zoom in and out in on the origin
    double rcenter_y = (-center_y*0.53) * x_size;
    double rcenter_x = (center_x * x_size);
    y = 0.53 * y;


    double real_y = ((y-rcenter_y)* zoom);
    double real_x = (x-rcenter_x) * zoom;

    real_y += x_size /2;
    real_x += x_size /2;


    putChar((int) real_x,(int) real_y, r,g,b, myChar,true);
}








