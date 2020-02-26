//
// Created by peter on 8/21/2018.
//

#ifndef ROVERRUCKUS_RANKED_TEL_CHAR_H
#define ROVERRUCKUS_RANKED_TEL_CHAR_H


#include "tel_char.h"

class ranked_tel_char {
public:
    tel_char* myTelChar;
    int colorDiffScore = 0;

    ranked_tel_char(tel_char* whoAmI, int score);
};


#endif //ROVERRUCKUS_RANKED_TEL_CHAR_H
