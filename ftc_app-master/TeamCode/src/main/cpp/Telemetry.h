#ifndef MASTER_TELEMETRY_H
#define MASTER_TELEMETRY_H
#include <vector>
#include <string>
#include <opencv2/core/mat.hpp>
#include "tel_char.h"

using namespace std;
using namespace cv;

class Telemetry {
public:
    //this will contain all the characters for the "image"
    vector<vector<tel_char>> allChars;//will be converted to strings when the user requests for drawing


    Telemetry(int size_x, int size_y);

    static int x_size;
    static int y_size;

    double center_x = 0;
    double center_y = 0;
    double zoom = 0;

    string * getString(int line);

    void clear();

    void drawField(double center_x, double center_y, double zoom);

    void drawRobot(double robotX, double robotY, double robotAng);

    void drawFPV(Mat *displayMe);

    void drawLine(int x1, int y1, int x2, int y2, int r, int g, int b, char myChar);

    void drawLineScaled(double x1, double y1, double x2, double y2, int r, int g, int b, char myChar);

    void putChar(int x, int y, int r, int g, int b, char myChar, bool isSignificant);


    string* convToHex(int r,int g, int b);

    bool compareByLength(const tel_char& struct1, const tel_char& struct2){
        return struct1.r > struct2.r;
    }

    void putCharField(double x, double y, int r, int g, int b, char myChar);
};


#endif //MASTER_TELEMETRY_H
