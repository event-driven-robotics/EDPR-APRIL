#include <signal.h>
#include <opencv2/opencv.hpp>
#include "erosdirect.h"

volatile sig_atomic_t isClosing = 0;

int main(int argc, char* argv[])
{
    signal(SIGINT, [](int signum){isClosing = 1;});
    EROSdirect erosdirect;
    if(!erosdirect.start(0.5, 0.01)) 
    {
        return -1;
    }

    for(;;)
    {
        cv::imshow("", erosdirect.eros.getSurface());
        char c = cv::waitKey(2);
        if(c == '\e')
            isClosing = 1;
        if (isClosing) 
            break;
    }

    erosdirect.stop();

    return 0;

}