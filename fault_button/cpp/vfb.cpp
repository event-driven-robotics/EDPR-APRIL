/* 
Author: Arren Glover
 */

#include <yarp/os/all.h>
#include <event-driven/core.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

using std::vector;
using yarp::os::Value;
using yarp::os::Network;

class vbfApp : public yarp::os::RFModule
{

private:

    //event reading
    ev::window<ev::AE> input_events;
    cv::Size img_size{{640, 480}};

    //parameters
    double k{0.2};       // seconds in window
    double p{0.01};      // detection rate
    double T{2e6};       // threshold events/second

    enum state_name{DISPLAY, FBR_DRAW, MONITOR, FINISHED};
    state_name state{DISPLAY};
    cv::Mat img, mask;

    typedef struct circle_parameters {
        cv::Point c;
        int       r;
        bool      d;
    } circle_parameters;

    circle_parameters c_fbr{{0,0},0,false};

public:
    bool configure(yarp::os::ResourceFinder &rf) override
    {

        // =====SET UP YARP=====
        if (!yarp::os::Network::checkNetwork(2.0))
        {
            std::cout << "Could not connect to YARP" << std::endl;
            return false;
        }

        // set the module name used to name ports
        setName((rf.check("name", Value("/visual-fault-button")).asString()).c_str());

        if (!input_events.open(getName("/AE:i")))
        {
            yError() << "Could not open events input port";
            return false;
        }

        // =====READ PARAMETERS=====

        k = rf.check("k", Value(0.001)).asFloat64();
        p = rf.check("p", Value(0.001)).asFloat64();
        T = rf.check("T", Value(5e5)).asFloat64();

        // ===== TRY DEFAULT CONNECTIONS =====
        Network::connect("/file/ch0dvs:o",  getName("/AE:i"), "fast_tcp");
        Network::connect("/file/atis/AE:o", getName("/AE:i"), "fast_tcp");
        Network::connect("/atis3/AE:o",     getName("/AE:i"), "fast_tcp");

        img = cv::Mat(img_size, CV_8UC3); img = 0;
        mask = cv::Mat(img_size, CV_8U); mask = 0;
        cv::namedWindow(getName(),  cv::WINDOW_KEEPRATIO);
        cv::resizeWindow(getName(), {640, 480});


        return true;
    }

    double getPeriod() override
    {
        return p;
    }

    bool interruptModule() override
    {
        // if the module is asked to stop ask the asynchronous thread to stop
        input_events.stop();
        return true;
    }

    bool close() override
    {
        // when the asynchronous thread is asked to stop, close ports and do other clean up
        return true;
    }

    static void mouseCall(int event, int x, int y, int flags, void* param)
    {
        circle_parameters *fbr = reinterpret_cast<circle_parameters*>(param);

        if(event == cv::EVENT_LBUTTONDOWN) {
            fbr->r = 0;
            fbr->c = {x, y};
            fbr->d = true;
        } else if(event == cv::EVENT_MOUSEMOVE && fbr->d) {
            fbr->r = sqrt((fbr->c.x-x)*(fbr->c.x-x) + (fbr->c.y-y)*(fbr->c.y-y));
        } else if(event == cv::EVENT_LBUTTONUP) {
            fbr->d = false;
        }
    }

    void makeMask()
    {
        for(int x = 0; x < img_size.width; x++) {
            for(int y = 0; y < img_size.height; y++) {
                int dist = sqrt((x-c_fbr.c.x)*(x-c_fbr.c.x)+(y-c_fbr.c.y)*(y-c_fbr.c.y));
                if(dist < c_fbr.r)
                    mask.at<char>(y, x) = 1;
            }
        }
    }
    
    void display()
    {
        ev::info stat = input_events.readSlidingWinT(0.1);

        img = 0;
        for(auto &v : input_events)
            img.at<cv::Vec3b>(v.y, v.x) = {128, 128, 128};
        
        cv::Mat img_display; img.copyTo(img_display);
        cv::putText(img_display, "Press SPACE when environment is clear", cv::Point(img_size.width*0.05, img_size.height*0.95), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});

        cv::imshow(getName(), img_display);
        char c = cv::waitKey(1);

        if(c == '\e')
            state = FINISHED;
        else if(c == ' ') {
            cv::setMouseCallback(getName(), mouseCall, &c_fbr);
            state = FBR_DRAW;
        }
    }

    void fbr_draw()
    {
        cv::Mat img_display; img.copyTo(img_display);
        
        cv::circle(img_display, c_fbr.c, c_fbr.r, {120, 10, 10}, -1);
        cv::circle(img_display, c_fbr.c, c_fbr.r, {255, 0, 0}, 4);

        cv::putText(img_display, "DRAW Fault Region. Click and drag. " + std::to_string(c_fbr.r), cv::Point(img_size.width*0.05, img_size.height*0.95), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});

        cv::imshow(getName(), img_display);
        char c = cv::waitKey(1);
        if(c == '\e')
            state = FINISHED;
        else if(c == ' ') {
            cv::setMouseCallback(getName(), nullptr);
            makeMask();
            state = MONITOR;
        }
    }

    void monitor()
    {

        ev::info stat = input_events.readSlidingWinT(k);

        int count = 0;

        img = 0;
        cv::circle(img, c_fbr.c, c_fbr.r, {50, 50, 50},   -1);
        cv::circle(img, c_fbr.c, c_fbr.r, {120, 120, 120}, 4);
        for(auto &v : input_events) {
            if(mask.at<char>(v.y, v.x)) {
                img.at<cv::Vec3b>(v.y, v.x) = {0, 200, 0};
                count++;
            } else {
                img.at<cv::Vec3b>(v.y, v.x) = {128, 128, 128};
            }
        }
        if(count > T * k){
            cv::circle(img, c_fbr.c, c_fbr.r, {0, 0, 255}, 6);
            for(auto &v : input_events) {
            if(mask.at<char>(v.y, v.x)) {
                img.at<cv::Vec3b>(v.y, v.x) = {0, 0, 200};
                count++;
            }
        }
        }

        cv::putText(img, "Monitoring Visual Fault Button ", cv::Point(img_size.width*0.05, img_size.height*0.95), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});

        cv::imshow(getName(), img);
        char c = cv::waitKey(1);

        if(c == '\e')
            state = FINISHED;
    }

    // synchronous thread
    bool updateModule() override
    {
        switch(state) 
        {
            case(DISPLAY):
                display(); break;
            case(FBR_DRAW):
                fbr_draw(); break;
            case(MONITOR):
                monitor(); break;
            case(FINISHED):
                return false;
        }


        return true;
    }

};

int main(int argc, char *argv[])
{
    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.configure(argc, argv);

    yInfo() << "I started";

    /* create the module */
    vbfApp instance;
    return instance.runModule(rf);
}