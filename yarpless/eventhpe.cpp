#pragma once

#include <signal.h>
#include <opencv2/opencv.hpp>
#include "erosdirect.h"

volatile sig_atomic_t flag = 0;
void my_function(int sig){ // can be called asynchronously
  flag = 1; // set flag
}

int main(int argc, char* argv[])
{

    signal(SIGINT, my_function);
    EROSdirect erosdirect;
    if(!erosdirect.start(0.5, 0.01)) 
    {
        return -1;
    }

    for(;;)
    {
        cv::imshow("", erosdirect.eros.getSurface());
        cv::waitKey(2);
        if (flag) 
        {  // my action when signal set it 1
            printf("\n Signal caught!\n");
            break;
        }
    }
    erosdirect.stop();

    return 0;

}