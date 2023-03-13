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

class externalDetector
{
private:
    double period{0.1}, tic{0.0};
    bool waiting{false};

    BufferedPort<ImageOf<PixelMono>> output_port;
    BufferedPort<Bottle> input_port;

public:
    bool init(std::string output_name, std::string input_name, double rate)
    {
        if (!output_port.open(output_name))
            return false;

        if (!input_port.open(input_name))
            return false;

        period = 1.0 / rate;
        return true;
    }
    void close()
    {
        output_port.close();
        input_port.close();
    }

    bool update(cv::Mat latest_image, double latest_ts, hpecore::stampedPose &previous_skeleton)
    {
        // send an update if the timer has elapsed
        if ((!waiting && latest_ts - tic > period) || (latest_ts - tic > 2.0))
        {
            static cv::Mat cv_image;
            cv::GaussianBlur(latest_image, cv_image, cv::Size(5, 5), 0, 0);
            output_port.prepare().copy(yarp::cv::fromCvMat<PixelMono>(cv_image));
            output_port.write();
            tic = latest_ts;
            waiting = true;
        }

        // read a ready data
        Bottle *mn_container = input_port.read(false);
        if (mn_container)
        {
            previous_skeleton.pose = hpecore::extractSkeletonFromYARP<Bottle>(*mn_container);
            previous_skeleton.timestamp = tic;
            previous_skeleton.delay = latest_ts - tic;
            waiting = false;
        }

        return mn_container != nullptr;
    }
};

class delayedGT
{
private:
    bool delay{false};
    double rate{10};
    BufferedPort<Bottle> input_port;
    hpecore::stampedPose internal{0.0, -1.0, {0}};

public:
    bool init(std::string input_name, double rate, bool delay)
    {
        if (!input_port.open(input_name))
            return false;

        this->delay = delay;
        this->rate = rate;
        return true;
    }
    void close()
    {
        input_port.close();
    }

    bool update(double latest_ts, hpecore::stampedPose &previous_skeleton)
    {
        Bottle *gt_container = input_port.read(false);
        if (gt_container && (rate * (latest_ts - internal.timestamp) > 1.0))
        {
            // if we have delay set the previous result to be returned
            if (delay)
                previous_skeleton = internal;

            // grab the new skeleton and set the timestamp to now
            internal.pose = hpecore::extractSkeletonFromYARP<Bottle>(*gt_container);
            internal.timestamp = latest_ts;

            // if we don't delay set the current result to be returned
            if (!delay)
                previous_skeleton = internal;

            // the delay is the difference between now and returned timestamp
            previous_skeleton.delay = latest_ts - previous_skeleton.timestamp;
            return true;
        }
        return false;
    }
};

class isaacHPE : public RFModule
{

private:
    // event reading
    std::thread asynch_thread;
    ev::window<ev::AE> input_events;

    // detection handlers
    externalDetector mn_handler;
    delayedGT gt_handler;

    // velocity and fusion
    hpecore::pwvelocity pw_velocity;
    hpecore::multiJointLatComp state;

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
    bool alt_view{false}, pltVel{false}, pltDet{false}, gpu{false}, ros{false};
    bool latency_compensation{true}, delay{false};
    double th_period{0.01};

    // ros
    yarp::os::Node *ros_node{nullptr};
    yarp::os::Publisher<yarp::rosmsg::Vjxoutput> ros_publisher;
    yarp::rosmsg::Vjxoutput ros_output;

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
        gpu = rf.check("gpu") && rf.check("gpu", Value(true)).asBool();
        ros = rf.check("ros") && rf.check("ros", Value(true)).asBool();
        detF = rf.check("detF", Value(10)).asInt32();

        image_size = cv::Size(rf.check("w", Value(640)).asInt32(),
                              rf.check("h", Value(480)).asInt32());
        roiSize = rf.check("roi", Value(20)).asInt32();

        double procU = rf.check("pu", Value(1e-1)).asFloat64();
        double measUD = rf.check("muD", Value(1e-4)).asFloat64();
        double measUV = rf.check("muV", Value(0)).asFloat64();
        latency_compensation = rf.check("use_lc") && rf.check("use_lc", Value(true)).asBool();
        double lc = latency_compensation ? 1.0 : 0.0;
        scaler = rf.check("sc", Value(12.5)).asFloat64();

        // ===== SET UP DETECTOR METHOD =====
        if (use_gt)
        {
            if (!gt_handler.init(getName("/gt:i"), detF, delay))
            {
                yError() << "Could not open input port";
                return false;
            }
        }

        if (movenet)
        {
            // run python code for movenet
            if (gpu)
                int r = system("python3 /usr/local/src/hpe-core/example/movenet/movenet_online.py --gpu &");
            else
                int r = system("python3 /usr/local/src/hpe-core/example/movenet/movenet_online.py &");
            while (!yarp::os::NetworkBase::exists("/movenet/sklt:o"))
                sleep(1);
            yInfo() << "MoveEnet started correctly";
            if (!mn_handler.init(getName("/eros:o"), getName("/movenet:i"), detF))
            {
                yError() << "Could not open movenet ports";
                return false;
            }
        }

        // ===== SET UP INTERNAL VARIABLE/DATA STRUCTURES =====

        // shared images
        vis_image = cv::Mat(image_size, CV_8UC3, cv::Vec3b(0, 0, 0));
        edpr_logo = cv::imread("/usr/local/src/wp5-hpe/edpr_logo.png");

        // velocity estimation
        pw_velocity.setParameters(image_size, 7, 0.3, 0.01);

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

        // * DPH19
        Network::connect("/file/ch3dvs:o", getName("/AE:i"), "fast_tcp");

        cv::namedWindow("edpr-april", cv::WINDOW_NORMAL);
        cv::resizeWindow("edpr-april", image_size);

        // set-up ROS interface
        if (ros)
        {
            ros_node = new yarp::os::Node("/APRIL");
            if (!ros_publisher.topic(getName("/output2ros")))
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
        mn_handler.close();
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
        pw_velocity.queryEROS().convertTo(eros8, CV_8U, 255);
        cv::cvtColor(eros8, img, cv::COLOR_GRAY2BGR);
        cv::GaussianBlur(img, img, cv::Size(5, 5), 0, 0);
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

        vis_image.setTo(cv::Vec3b(0, 0, 0));

        // plot skeletons
        if (pltDet)
            hpecore::drawSkeleton(canvas, skeleton_detection, {255, 0, 0}, 3);

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
            bool was_detected = false;
            if (use_gt)
            {
                was_detected = gt_handler.update(tnow, detected_pose);
            }
            else if (movenet)
            {
                static cv::Mat eros8;
                pw_velocity.queryEROS().convertTo(eros8, CV_8U, 255);
                was_detected = mn_handler.update(eros8, tnow, detected_pose);
            }

            if (was_detected && hpecore::poseNonZero(detected_pose.pose))
            {
                skeleton_detection = detected_pose.pose;
                latency = detected_pose.delay;
                if (state.poseIsInitialised())
                    state.updateFromPosition(skeleton_detection, detected_pose.timestamp);
                else
                    state.set(skeleton_detection, tnow);
            }

            // ---------- VELOCITY ----------
            // read events
            event_stats = input_events.readAll(false);
            if (event_stats.count == 0)
                continue;

            // update images
            for (auto &v : input_events)
            {
                if (v.p)
                    vis_image.at<cv::Vec3b>(v.y, v.x) = cv::Vec3b(64, 150, 90);
                else
                    vis_image.at<cv::Vec3b>(v.y, v.x) = cv::Vec3b(32, 82, 50);
            }

            pw_velocity.update(input_events.begin(), input_events.end(), event_stats.timestamp);

            // only update velocity if the pose is initialised
            if (!state.poseIsInitialised())
                continue;

            skel_vel = pw_velocity.query(state.query(), roiSize, 2, state.queryVelocity());

            // this scaler was thought to be from timestamp misconversion.
            // instead we aren't sure why it is needed.
            for (int j = 0; j < 13; j++) // (F) overload * to skeleton13
                skel_vel[j] = skel_vel[j] * scaler;

            state.setVelocity(skel_vel);
            state.updateFromVelocity(skel_vel, event_stats.timestamp);

            skelwriter.write({event_stats.timestamp, latency, state.query()});
            velwriter.write({event_stats.timestamp, latency, skel_vel});

            if (ros)
            {
                // format skeleton to ros output
                sklt_out.clear();
                vel_out.clear();
                for (int j = 0; j < 13; j++)
                {
                    sklt_out.push_back(skeleton_detection[j].u);
                    sklt_out.push_back(skeleton_detection[j].v);
                    vel_out.push_back(skel_vel[j].u);
                    vel_out.push_back(skel_vel[j].v);
                }
                ros_output.timestamp = tnow;
                ros_output.pose = sklt_out;
                ros_output.velocity = vel_out;
                // publish data
                ros_publisher.prepare() = ros_output;
                ros_publisher.write();
            }
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
    isaacHPE instance;
    return instance.runModule(rf);
}