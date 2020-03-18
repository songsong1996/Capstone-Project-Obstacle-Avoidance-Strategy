//
// Created by songsong on 19-5-15.
//
#include "kalman.h"

Point mousePosition;


void mouseEvent(int event, int x, int y, int flags, void *param )
{
    if (event==EVENT_MOUSEMOVE) {
        mousePosition = Point(x,y);
    }
}


int main(int, char**)
{
    kalman_filter kf;
    Mat image(winHeight,winWidth,CV_8UC3,Scalar(0));


    namedWindow("kalman");
    setMouseCallback("kalman",mouseEvent);


    while(1) {
        kf.update(mousePosition);
        kf.draw(image);
    }


}