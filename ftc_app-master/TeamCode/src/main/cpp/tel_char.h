//
// Created by peter on 8/17/18.
//

#ifndef MASTER_TEL_CHAR_H
#define MASTER_TEL_CHAR_H

#include <string>

using namespace std;

class tel_char {
public:
    tel_char(int red, int green, int blue, char c);

    int r = 0;
    int g = 0;
    int b = 0;
    char myChar = ' ';

    int colorChange = 0;//the sum of the absolute differences of this char's color vs the last

    bool isSignificant = false;//if color will be updated for this char


};


#endif //MASTER_TEL_CHAR_H
