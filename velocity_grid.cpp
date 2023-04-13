/*
Author: Franco Di Pietro, Arren Glover
 */

#include <yarp/cv/Cv.h>
#include <yarp/os/all.h>
#include <yarp/sig/Image.h>
#include <event-driven/core.h>
#include <hpe-core/utility.h>
#include <hpe-core/motion_estimation.h>
#include <hpe-core/fusion.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "yarp/rosmsg/Vjxoutput.h"

using namespace yarp::os;
using namespace yarp::sig;
using std::vector;

class velGrid : public RFModule
{

private:
    // event reading
    std::thread asynch_thread;
    ev::window<ev::AE> input_events;

    // detection handlers

    // velocity and fusion
    hpecore::pwvelocity pw_velocity;
    hpecore::surfacedVelocity sf_velocity;
    hpecore::queuedVelocity q_velocity;
    hpecore::tripletVelocity trip_velocity;
    hpecore::multiJointLatComp state;
    // hpecore::stateEstimator state;

    // internal data structures
    hpecore::skeleton13 skeleton_gt{0};
    hpecore::skeleton13 skeleton_detection{0};

    cv::Size image_size;
    cv::Mat vis_image;
    cv::Mat edpr_logo;

    // recording results
    hpecore::writer skelwriter, velwriter;
    cv::VideoWriter output_video;

    // parameters
    bool movenet{false}, use_gt{false};
    int detF{10}, roiSize{20};
    double scaler{12.5};
    bool alt_view{false}, pltVel{false}, pltDet{false}, pltTra{false}, gpu{false}, ros{false};
    bool vpx{false}, vsf{false}, ver{false}, vcr{false}, vqu{false}, vtr{false};
    bool latency_compensation{true}, delay{false};
    double th_period{0.00005};
    bool dhp19{false};
    bool pltRoi{false};
    bool showGrid{false};
    cv::Scalar colors[13] = {{0, 0, 180}, {0, 180, 0}, {0, 0, 180},
                            {180, 180, 0}, {180, 0, 180}, {0, 180, 180},
                            {120, 0, 180}, {120, 180, 0}, {0, 120, 180},
                            {120, 120, 180}, {120, 180, 120}, {120, 120, 180}, {120, 120, 120}};

    // ros
    yarp::os::Node *ros_node{nullptr};
    yarp::os::Publisher<yarp::rosmsg::Vjxoutput> ros_publisher;
    yarp::rosmsg::Vjxoutput ros_output;
    std::vector<hpecore::joint> grid;
    int rows{10}, cols{10};
    double delta;

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
        setName((rf.check("name", Value("/edpr_april")).asString()).c_str());

        if (!input_events.open(getName("/AE:i")))
        {
            yError() << "Could not open events input port";
            return false;
        }

        // =====READ PARAMETERS=====
        movenet = rf.check("movenet") && rf.check("movenet", Value(true)).asBool();
        use_gt = !movenet ? true : false;
        delay = rf.check("delay") && rf.check("delay", Value(true)).asBool();
        alt_view = rf.check("alt_view") && rf.check("alt_view", Value(true)).asBool();
        pltDet = rf.check("pltDet") && rf.check("pltDet", Value(true)).asBool();
        pltTra = rf.check("pltTra") && rf.check("pltTra", Value(true)).asBool();
        pltRoi = rf.check("pr") && rf.check("pr", Value(true)).asBool();
        gpu = rf.check("gpu") && rf.check("gpu", Value(true)).asBool();
        ros = rf.check("ros") && rf.check("ros", Value(true)).asBool();
        detF = rf.check("detF", Value(10)).asInt32();
        dhp19 = rf.check("dhp19") && rf.check("dhp19", Value(true)).asBool();
        image_size = cv::Size(rf.check("w", Value(640)).asInt32(),
                              rf.check("h", Value(480)).asInt32());
        roiSize = rf.check("roi", Value(20)).asInt32();
        double procU = rf.check("pu", Value(1e-1)).asFloat64();
        double measUD = rf.check("muD", Value(1e-4)).asFloat64();
        double measUV = rf.check("muV", Value(0)).asFloat64();
        latency_compensation = rf.check("use_lc") && rf.check("use_lc", Value(true)).asBool();
        double lc = latency_compensation ? 1.0 : 0.0;
        scaler = rf.check("sc", Value(12.5)).asFloat64();

        showGrid = rf.check("grid") && rf.check("grid", Value(true)).asBool();
        rows = rf.check("r", Value(10)).asFloat64();
        cols = rf.check("c", Value(10)).asFloat64();
        grid.reserve(rows*cols);
        for(int i=0; i<rows*cols; i++)
        {
            grid[i].u = 0.0;
            grid[i].v = 0.0;
        }
        delta = rf.check("d", Value(1.0)).asFloat64();

        // std::fill(grid.begin(), grid.end(), 0);

        pltDet = true;
        pltTra = true;

        // ===== SELECT VELOCITY ESTIMATION METHOD =====
        std::string method = rf.check("ve", Value("")).asString();
        if(!method.compare("px"))
        {
            vpx = true;
            yInfo() << "Velocity estimation method = Pixel-wise";
        }
        else if(!method.compare("surf"))
        {
            vsf = true;
            scaler = 2;
            yInfo() << "Velocity estimation method = Past surfaces";
        }
        else if(!method.compare("err"))
        {
            ver = true;
            scaler = 2;
            yInfo() << "Velocity estimation method = Error to previous velocity";
        }
        else if(!method.compare("circle"))
        {
            vcr = true;
            scaler = 1;
            yInfo() << "Velocity estimation method = Error to observation circle";
        }    
        else if(!method.compare("q"))
        {
            vqu = true;
            scaler = 30;
            yInfo() << "Velocity estimation method = Queues of events";
        }
        else if(!method.compare("trip"))
        {
            vtr = true;
            scaler = 1;
            yInfo() << "Velocity estimation method = Triplet";
        }
        if(!method.length()) yInfo() << "Velocity estimation method = NONE";

        if(dhp19)
        {
            image_size = cv::Size(346, 260);
            roiSize = 12;
        }

        // ===== SET UP DETECTOR METHOD =====


        // ===== SET UP INTERNAL VARIABLE/DATA STRUCTURES =====

        // shared images
        vis_image = cv::Mat(image_size, CV_8UC3, cv::Vec3b(0, 0, 0));
        edpr_logo = cv::imread("/usr/local/src/wp5-hpe/edpr_logo.png");

        // velocity estimation
        pw_velocity.setParameters(image_size, 7, 0.3, 0.01);
        sf_velocity.setParameters(roiSize, 2, 8, 1000, image_size);
        q_velocity.setParameters(roiSize, 2, 8, 1000);
        trip_velocity.setParameters(roiSize, 1, image_size);

        // fusion
        if (!state.initialise({procU, measUD, measUV, lc}))
        {
            yError() << "Not KF initialized";
            return false;
        }

        // ===== SET UP RECORDING =====
        if (rf.check("filepath"))
        {
            std::string filepath = rf.find("filepath").asString();
            if (skelwriter.open(filepath))
                yInfo() << "saving data to:" << filepath;
        }
        if (rf.check("velpath"))
        {
            std::string velpath = rf.find("velpath").asString();
            if (velwriter.open(velpath))
                yInfo() << "saving velocity data to:" << velpath;
        }

        if (rf.check("v"))
        {
            std::string videopath = rf.find("v").asString();
            if (!output_video.open(videopath,
                                   cv::VideoWriter::fourcc('H', '2', '6', '4'),
                                   (int)(0.1 / th_period),
                                   image_size,
                                   true))
            {
                yError() << "Could not open video writer!!";
                return false;
            }
        }

        // ===== TRY DEFAULT CONNECTIONS =====
        Network::connect("/file/ch0dvs:o", getName("/AE:i"), "fast_tcp");
        Network::connect("/atis3/AE:o", getName("/AE:i"), "fast_tcp");
        Network::connect("/file/ch2GT50Hzskeleton:o", getName("/gt:i"), "fast_tcp");
        Network::connect("/movenet/sklt:o", getName("/movenet:i"), "fast_tcp");
        Network::connect("/zynqGrabber/AE:o", getName("/AE:i"), "fast_tcp");
        Network::connect(getName("/eros:o"), "/movenet/img:i", "fast_tcp");
        Network::connect("/file/atis/AE:o", getName("/AE:i"), "fast_tcp");
        // * DPH19
        Network::connect("/file/ch3dvs:o", getName("/AE:i"), "fast_tcp");
        Network::connect("/file/ch3GTskeleton:o", getName("/gt:i"), "fast_tcp");
        // * Mustard bottle
        Network::connect("/file/leftdvs:o", getName("/AE:i"), "fast_tcp");
        

        cv::namedWindow("edpr-april", cv::WINDOW_NORMAL);
        cv::resizeWindow("edpr-april", image_size);

        // set-up ROS interface
        if (ros)
        {
            ros_node = new yarp::os::Node("/APRIL");
            if (!ros_publisher.topic("/isim/neuromorphic_camera/data"))
            {
                yError() << "Could not open ROS output publisher";
                return false;
            }
            else
                yInfo() << "ROS output publisher: OK";
        }

        asynch_thread = std::thread([this]
                                    { this->run_opixels(); });

        return true;
    }

    double getPeriod() override
    {
        // run the module as fast as possible. Only as fast as new images are
        // available and then limited by how fast OpenPose takes to run
        return th_period;
    }

    bool interruptModule() override
    {
        // if the module is asked to stop ask the asynchronous thread to stop
        input_events.stop();
        skelwriter.close();
        velwriter.close();
        output_video.release();
        asynch_thread.join();
        return true;
    }

    bool close() override
    {
        // when the asynchronous thread is asked to stop, close ports and do other clean up
        return true;
    }

    void drawEROS(cv::Mat img)
    {
        cv::Mat eros8;
        if(vpx) pw_velocity.queryEROS().convertTo(eros8, CV_8U, 255);
        // if(vpx) pw_velocity.queryEROS().copyTo(eros8);
        // if(vsf || ver || vcr) sf_velocity.queryEROS().convertTo(eros8, CV_8U, 255);
        if(vsf || ver || vcr) sf_velocity.querySAE().convertTo(eros8, CV_8U, 255);
        cv::cvtColor(eros8, img, cv::COLOR_GRAY2BGR);
        cv::GaussianBlur(img, img, cv::Size(5, 5), 0, 0);
    }

    void drawROI(cv::Mat img)
    {
        for(int i=0; i<13 ; i++)
        {
            float cx = state.query()[i].u;
            float cy = state.query()[i].v;
            cv::Point2d p1(cx-roiSize/2, cy-roiSize/2);
            cv::Point2d p2(cx+roiSize/2, cy+roiSize/2);
            cv::rectangle(img, cv::Rect(p1, p2), colors[i], 1);
            
        }
    }


    // synchronous thread
    bool updateModule() override
    {
        static cv::Mat canvas = cv::Mat(image_size, CV_8UC3);
        canvas.setTo(cv::Vec3b(0, 0, 0));

        
        // plot the image
        // check if we plot events or alternative (PIM or EROS)
        if (alt_view)
            drawEROS(canvas);
        else // events
            vis_image.copyTo(canvas);
        if(pltRoi)
            drawROI(canvas);

        vis_image.setTo(cv::Vec3b(0, 0, 0));


        // for(int i=0; i<rows; i++)
        // {
        //     for(int j=0; j<cols; j++)
        //     {
        //         std::cout << grid[i*cols+j].u << ", " << grid[i*cols+j].v << "\t";
        //     }
        //     std::cout << std::endl;
        // }
        // std::cout << std::endl;
        hpecore::drawGrid(canvas, grid, rows, cols, {102, 255, 255}, 1, showGrid, delta);
        // int width = canvas.cols/cols;
        // int height = canvas.rows/rows;
        // for(int i=0; i<rows; i++)
        // {
        //     for(int j=0; j<cols; j++)
        //     {
        //         std::stringstream ss;
        //         ss << std::fixed << std::setprecision(1) << i*cols+j;
        //         std::string mystring = ss.str();

        //         cv::putText(canvas, //target image
        //             mystring, //text
        //             cv::Point(j*width+width/2, i*height+height/2), //top-left position
        //             cv::FONT_HERSHEY_DUPLEX,
        //             0.6,
        //             CV_RGB(150, 150, 150), //font color
        //             2);
        //     }
        // }
        // plot skeletons
        if (pltDet)
            hpecore::drawSkeleton(canvas, skeleton_detection, {255, 0, 0}, 3);
        if (pltTra)
            hpecore::drawSkeleton(canvas, state.query(), {0, 0, 255}, 3);

        if (pltVel)
        {
            hpecore::skeleton13_vel jv = state.queryVelocity();
            for (int j = 0; j < 13; j++) // (F) overload * to skeleton13
                jv[j] = jv[j] * 0.1;

            hpecore::drawVel(canvas, state.query(), jv, {255, 255, 102}, 2);
        }

        if (!edpr_logo.empty())
        {
            static cv::Mat mask;
            cv::cvtColor(edpr_logo, mask, CV_BGR2GRAY);
            edpr_logo.copyTo(canvas, mask);
        }

        if (output_video.isOpened())
            output_video << canvas;

        // std::stringstream ss;
        // ss << std::fixed << std::setprecision(1) << scaler;
        // std::string mystring = ss.str();

        // cv::putText(canvas, //target image
        //     mystring, //text
        //     cv::Point(canvas.cols - 60, canvas.rows - 30), //top-left position
        //     cv::FONT_HERSHEY_DUPLEX,
        //     0.6,
        //     CV_RGB(150, 150, 150), //font color
        //     2);

        cv::imshow("edpr-april", canvas);
        char key_pressed = cv::waitKey(10);
        if (key_pressed > 0)
        {
            switch (key_pressed)
            {
            case 'v':
                pltVel = !pltVel;
                break;
            case 'd':
                pltDet = !pltDet;
                break;
            case 'e':
                alt_view = !alt_view;
                break;
            case 't':
                pltTra = !pltTra;
                break;
            case 'r':
                pltRoi = !pltRoi;
                break;
            case 'g':
                showGrid = !showGrid;
                break;
            case ',':
                if(roiSize>0)
                    roiSize--;
                sf_velocity.setParameters(roiSize, 2, 8, 1000, image_size);
                q_velocity.setParameters(roiSize, 2, 8, 1000);
                break;
            case '.':
                if(roiSize<image_size.height/2)
                    roiSize++;
                sf_velocity.setParameters(roiSize, 2, 8, 1000, image_size);
                q_velocity.setParameters(roiSize, 2, 8, 1000);
                break;
            case '[':
                if(scaler>0)
                    scaler-=0.5;
                break;
            case ']':
                if(scaler<30)
                    scaler+=0.5;
                break;
            case '\e':
                stopModule();
                break;
            }
        }
        return true;
    }

    void run_opixels()
    {
        hpecore::skeleton13_vel jv;
        hpecore::skeleton13_vel skel_vel = {0};
        ev::info event_stats = {0};
        double latency = 0.0;
        hpecore::stampedPose detected_pose;
        input_events.readPacket(true);
        double t0 = Time::now();
    std:
        vector<double> sklt_out, vel_out;

        while (!isStopping())
        {
            double tnow = Time::now() - t0;

            // ---------- DETECTIONS ----------
            

            // ---------- VELOCITY ----------
            // read events
            event_stats = input_events.readAll(false);
            if (event_stats.count == 0)
                continue;

            // update images
            for (auto &v : input_events)
            {
                // if (v.p)
                //     vis_image.at<cv::Vec3b>(v.y, v.x) = cv::Vec3b(64, 150, 90);
                // else
                //     vis_image.at<cv::Vec3b>(v.y, v.x) = cv::Vec3b(32, 82, 50);
                vis_image.at<cv::Vec3b>(v.y, v.x) = cv::Vec3b(150, 150, 150);
            }

            if(vpx) pw_velocity.update(input_events.begin(), input_events.end(), event_stats.timestamp);
            pw_velocity.update(input_events.begin(), input_events.end(), tnow);
            
            pw_velocity.query_grid(grid, event_stats.timestamp, rows, cols, 2);

            // this scaler was thought to be from timestamp misconversion.
            // instead we aren't sure why it is needed.
            for (int j = 0; j < grid.size(); j++) // (F) overload * to skeleton13
                grid[j] = grid[j] * scaler;
            
        }
    }
};

int main(int argc, char *argv[])
{
    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.configure(argc, argv);

    /* create the module */
    velGrid instance;
    return instance.runModule(rf);
}
