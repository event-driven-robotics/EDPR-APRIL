/* 
Author: Franco Di Pietro, Arren Glover
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

    enum state_name{DISPLAY, FBR_DRAW, MONITOR};
    state_name state{DISPLAY};

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

        k = rf.check("k", Value(0.2)).asFloat64();
        p = rf.check("p", Value(0.01)).asFloat64();
        T = rf.check("T", Value(2e6)).asFloat64();

        // ===== TRY DEFAULT CONNECTIONS =====
        Network::connect("/file/ch0dvs:o", getName("/AE:i"), "fast_tcp");
        Network::connect("/atis3/AE:o", getName("/AE:i"), "fast_tcp");

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

    //
    void display()
    {
        ev::info stat = input_events.readSlidingWinT(1.0);

        static cv::Mat img(img_size, CV_8U);
        
        
    }

    void fbr_draw()
    {

    }

    void monitor()
    {

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

    /* create the module */
    vbfApp instance;
    return instance.runModule(rf);
}