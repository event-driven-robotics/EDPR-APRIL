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
#include <hpe-core/motion.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "april_msgs/yarp/rosmsg/april_msgs/NChumanPose.h"
#include <yarp/rosmsg/sensor_msgs/Image.h>

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

    bool update(const cv::Mat &latest_image, double latest_ts, hpecore::stampedPose &previous_skeleton)
    {
        // send an update if the timer has elapsed
        if(latest_ts < tic) tic = latest_ts - 2.0;
        if ((!waiting && latest_ts - tic > period) || (latest_ts - tic > 2.0))
        {
            static cv::Mat cv_image;
            latest_image.convertTo(cv_image, CV_8U);
            cv::GaussianBlur(cv_image, cv_image, cv::Size(5, 5), 0, 0);
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

class APRIL_HPE : public RFModule
{

private:
    // event reading
    std::thread camera_handler_thread;
    std::thread hpe_thread;
    ev::window<ev::AE> input_events;
    

    // detection handlers
    externalDetector mn_handler;
    delayedGT gt_handler;
    hpecore::EROS eros_handler;
    hpecore::SAE sae_handler;
    hpecore::BIN binary_handler;

    // velocity and fusion
    hpecore::pwtripletvelocity velocity_estimator;
    hpecore::pwTripletVelocity pw_trip_velocity;
    hpecore::multiJointLatComp state;

    // internal data structures
    hpecore::skeleton13 skeleton_gt{0};
    hpecore::skeleton13 skeleton_detection{0};

    cv::Size image_size;
    cv::Mat edpr_logo;

    // parameters
    int detF{10}, roiSize{20};
    bool pltVel{false}, pltDet{false}, pltTra{false};
    int alt_view{0};
    bool latency_compensation{true};
    double scaler{1.0};
    double th_period{0.01}, thF{100.0};
    bool pltRoi{false};
    cv::Scalar colors[13] = {{0, 0, 180}, {0, 180, 0}, {0, 0, 180},
                            {180, 180, 0}, {180, 0, 180}, {0, 180, 180},
                            {120, 0, 180}, {120, 180, 0}, {0, 120, 180},
                            {120, 120, 180}, {120, 180, 120}, {120, 120, 180}, {120, 120, 120}};
    bool started{false};
    double tnow;

    // ros 
    yarp::os::Node* ros_node{nullptr};
    yarp::os::Publisher<yarp::rosmsg::april_msgs::NChumanPose> ros_publisher;
    yarp::rosmsg::april_msgs::NChumanPose ros_output;
    typedef yarp::os::Publisher<yarp::rosmsg::sensor_msgs::Image> ImageTopicType;
    ImageTopicType publisherPort_eros, publisherPort_evs;

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
        pltDet = rf.check("pltDet") && rf.check("pltDet", Value(true)).asBool();
        pltTra = rf.check("pltTra") && rf.check("pltTra", Value(true)).asBool();
        pltRoi = rf.check("pr") && rf.check("pr", Value(true)).asBool();
        detF = rf.check("detF", Value(1)).asInt32();
        image_size = cv::Size(rf.check("w", Value(640)).asInt32(),
                              rf.check("h", Value(480)).asInt32());
        roiSize = rf.check("roi", Value(26)).asInt32();
        double procU = rf.check("pu", Value(1e-1)).asFloat64();
        double measUD = rf.check("muD", Value(1e-4)).asFloat64();
        double measUV = rf.check("muV", Value(0)).asFloat64();
        latency_compensation = rf.check("use_lc") && rf.check("use_lc", Value(true)).asBool();
        double lc = latency_compensation ? 1.0 : 0.0;
        thF = rf.check("thF", Value(100.0)).asFloat64();
        th_period = 1/thF;

        // pltDet = true;
        pltTra = true;
        

        int r = system("python3 /usr/local/src/hpe-core/example/movenet/movenet_online.py --gpu &");
        while (!yarp::os::NetworkBase::exists("/movenet/sklt:o"))
            sleep(1);
        yInfo() << "MoveEnet started correctly";
        if (!mn_handler.init(getName("/eros:o"), getName("/movenet:i"), detF))
        {
            yError() << "Could not open movenet ports";
            return false;
        }

        // ===== SET UP INTERNAL VARIABLE/DATA STRUCTURES =====

        // shared images
        eros_handler.init(image_size.width, image_size.height, 7, 0.3);
        binary_handler.init(image_size.width, image_size.height);
        sae_handler.init(image_size.width, image_size.height);

        edpr_logo = cv::imread("/usr/local/src/EDPR-APRIL/edpr_logo.png");
        
        //velocity estimation
        pw_trip_velocity.setParameters(roiSize, 1, image_size);

        // fusion
        if (!state.initialise({procU, measUD, measUV, lc}))
        {
            yError() << "Not KF initialized";
            return false;
        }

        // ===== TRY DEFAULT CONNECTIONS =====
        Network::connect("/file/ch0dvs:o", getName("/AE:i"), "fast_tcp");
        Network::connect("/atis3/AE:o", getName("/AE:i"), "fast_tcp");
        Network::connect("/file/ch2GT50Hzskeleton:o", getName("/gt:i"), "fast_tcp");
        Network::connect("/movenet/sklt:o", getName("/movenet:i"), "fast_tcp");
        Network::connect("/zynqGrabber/AE:o", getName("/AE:i"), "fast_tcp");
        Network::connect(getName("/eros:o"), "/movenet/img:i", "fast_tcp");
        Network::connect("/file/atis/AE:o", getName("/AE:i"), "fast_tcp");

        cv::namedWindow("edpr-april", cv::WINDOW_NORMAL);
        cv::resizeWindow("edpr-april", image_size);

        // set-up ROS interface
        ros_node = new yarp::os::Node("/APRIL");
        if (!ros_publisher.topic("/pem/neuromorphic_camera/data"))
        {
            yError() << "Could not open ROS pose output publisher";
            return false;
        }

        if (!publisherPort_eros.topic("/isim/neuromorphic_camera/eros"))
        {
            yError() << "Could not open ROS EROS output publisher";
            return false;
        }

        if (!publisherPort_evs.topic("/isim/neuromorphic_camera/evs"))
        {
            yError() << "Could not open ROS EVS output publisher";
            return false;
        }
        
        camera_handler_thread = std::thread([this]{ this->run_camera_interface(); });
        hpe_thread = std::thread([this]{ this->run_hpe(); });

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
        camera_handler_thread.join();
        hpe_thread.join();
        int r = system("killall python3");
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
        eros_handler.getSurface().convertTo(eros8, CV_8U);
        cv::GaussianBlur(eros8, eros8, {5, 5}, -1);
        cv::cvtColor(eros8, img, cv::COLOR_GRAY2BGR);
    }

    void drawEVENTS(cv::Mat &img)
    {
        cv::Mat eventsmono;
        binary_handler.getSurface().convertTo(eventsmono, CV_8U);
        cv::cvtColor(eventsmono, img, CV_GRAY2BGR);
    }

    void drawSAE(cv::Mat &img)
    {
        cv::Mat sae64, saemono;
        
        sae_handler.getSurface().copyTo(sae64);
        double maxval;
        cv::minMaxLoc(sae64, nullptr, &maxval);
        sae64 -= (maxval - 1.0);  //show 2 seconds of surface
        //cv::threshold(sae64, sae64, 0, 0, cv::THRESH_BINARY);
        sae64.convertTo(saemono, CV_8U, 255.0);
        cv::cvtColor(saemono, img, CV_GRAY2BGR);
    }

    void drawROI(cv::Mat img)
    {
        for(int i=0; i<13 ; i++)
        {
            float cx = state.query()[i].u;
            float cy = state.query()[i].v;
            cv::Point2d p1(cx-roiSize, cy-roiSize);
            cv::Point2d p2(cx+roiSize, cy+roiSize);
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
        if (alt_view == 0)
            drawEROS(canvas);
        else if(alt_view == 1)// events
            drawEVENTS(canvas);
        else if(alt_view == 2)
            drawSAE(canvas);

        //if(pltRoi)
        drawROI(canvas);

        // yarp::rosmsg::std_msgs::Header header;
        // std::uint32_t height;
        // std::uint32_t width;
        // std::string encoding;
        // std::uint8_t is_bigendian;
        // std::uint32_t step;
        // std::vector<std::uint8_t> data;

        static yarp::os::Stamp ystamp;
        ystamp.update();

        // publish images using ROS
        // EROS
        static cv::Mat cvEROS = cv::Mat(image_size, CV_8UC3);
        cvEROS.setTo(cv::Vec3b(0, 0, 0));
        drawEROS(cvEROS);
        auto yarpEROS = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(cvEROS);
        yarp::rosmsg::sensor_msgs::Image& rosEROS = publisherPort_eros.prepare();
        rosEROS.data.resize(yarpEROS.getRawImageSize());
        rosEROS.width = yarpEROS.width();
        rosEROS.height = yarpEROS.height();
        rosEROS.encoding = "8UC3";//yarp::dev::ROSPixelCode::yarp2RosPixelCode(yarpEROS.getPixelCode());
        rosEROS.step = yarpEROS.getRowSize();
        rosEROS.is_bigendian = 0;
        rosEROS.header.frame_id = "eros";
        rosEROS.header.seq = ystamp.getCount();
        rosEROS.header.stamp = ystamp.getTime();
        memcpy(rosEROS.data.data(), yarpEROS.getRawImage(), yarpEROS.getRawImageSize());
        publisherPort_eros.setEnvelope(ystamp);
        publisherPort_eros.write();

        // EV image
        static cv::Mat cvEVS;
        drawEVENTS(cvEVS);

        auto yarpEVS = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(cvEVS);
        yarp::rosmsg::sensor_msgs::Image& rosEVS = publisherPort_evs.prepare();
        rosEVS.data.resize(yarpEVS.getRawImageSize());
        rosEVS.width = yarpEVS.width();
        rosEVS.height = yarpEVS.height();
        rosEVS.encoding = "8UC3";//yarp::dev::ROSPixelCode::yarp2RosPixelCode(yarpEVS.getPixelCode());
        rosEVS.step = yarpEVS.getRowSize();
        rosEVS.is_bigendian = 0;
        rosEVS.header.frame_id = "eventimage";
        rosEVS.header.seq = ystamp.getCount();
        rosEVS.header.stamp = ystamp.getTime();
        memcpy(rosEVS.data.data(), yarpEVS.getRawImage(), yarpEVS.getRawImageSize());

        publisherPort_evs.setEnvelope(ystamp);
        publisherPort_evs.write();     

        binary_handler.getSurface().setTo(0.0);

        // plot skeletons
        if (pltDet)
            hpecore::drawSkeleton(canvas, skeleton_detection, {255, 0, 0}, 3);
        if (pltTra)
            hpecore::drawSkeleton(canvas, state.query(), {0, 0, 255}, 3);

        if (pltVel)
        {
            hpecore::skeleton13_vel jv = state.queryVelocity();
            //for (int j = 0; j < 13; j++) // (F) overload * to skeleton13
             //   jv[j] = jv[j] * 0.1;

            hpecore::drawVel(canvas, state.query(), jv, {255, 255, 102}, 2);
        }

        if (!edpr_logo.empty())
        {
            static cv::Mat mask;
            cv::cvtColor(edpr_logo, mask, CV_BGR2GRAY);
            edpr_logo.copyTo(canvas, mask);
        }

        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << scaler;
        std::string mystring = ss.str();

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
                ++alt_view %= 3;
                break;
            case 't':
                pltTra = !pltTra;
                break;
            case 'r':
                pltRoi = !pltRoi;
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

    void run_camera_interface()
    {
        while (!isStopping())
        {
            
            ev::info stats = input_events.readAll(true);
            tnow = stats.timestamp;

            //check a reset
            static double pts = 0;
            if(tnow < pts) {
                sae_handler.getSurface().setTo(0.0);
                binary_handler.getSurface().setTo(0.0);
                eros_handler.getSurface().setTo(0.0);
                state.reset();
                velocity_estimator.prev_update_ts = 0;
            }
            pts = tnow;

            //do the update of surfaces
            for(auto &v : input_events) {
                eros_handler.update(v.x, v.y);
                binary_handler.update(v.x, v.y);
                sae_handler.update(v.x, v.y, tnow);
            }
        }
    }

    void run_hpe()
    {

        while(!isStopping()) {
            //detection
            hpecore::stampedPose detected_pose;
            bool was_detected = mn_handler.update(eros_handler.getSurface(), tnow, detected_pose);
            if (was_detected && hpecore::poseNonZero(detected_pose.pose))
            {
                skeleton_detection = detected_pose.pose;
                //latency = detected_pose.delay;
                if (state.poseIsInitialised())
                    state.updateFromPosition(skeleton_detection, detected_pose.timestamp);
                else
                    state.set(skeleton_detection, tnow);
            }

            
            if (!state.poseIsInitialised())
               continue;

            //velocity
            auto jvs = velocity_estimator.multi_area_velocity(sae_handler.getSurface(), tnow, state.query(), roiSize);
            state.setVelocity(jvs);
            state.updateFromVelocity(jvs, tnow);
            //hpecore::print_skeleton<hpecore::skeleton13>(jvs);

            yarp::os::Time::delay(0.01);



        }

    }

    // void run_detection()
    // {
    //     double latency = 0.0;
    //     hpecore::stampedPose detected_pose;
    //     double t0 = Time::now();
    //     std:vector<double> sklt_out; 

    //     while (!isStopping())
    //     {
    //         tnow = Time::now() - t0;

    //         // ---------- DETECTIONS ----------
    //         bool was_detected = false;
    //         static cv::Mat eros8;
    //         pw_trip_velocity.queryEROS().convertTo(eros8, CV_8U, 255);
    //         was_detected = mn_handler.update(eros8, tnow, detected_pose);
            

    //         if (was_detected && hpecore::poseNonZero(detected_pose.pose))
    //         {
    //             skeleton_detection = detected_pose.pose;
    //             latency = detected_pose.delay;
    //             if (state.poseIsInitialised())
    //                 state.updateFromPosition(skeleton_detection, detected_pose.timestamp);
    //             else
    //                 state.set(skeleton_detection, tnow);
    //         }
    //     }
    // }

    // void run_opixels()
    // {
    //     hpecore::skeleton13_vel jv;
    //     hpecore::skeleton13_vel skel_vel = {0};
    //     ev::info event_stats = {0};
    //     double latency = 0.0;
    //     hpecore::stampedPose detected_pose;
    //     input_events.readPacket(true);
    //     double t0 = Time::now();
    //     std:vector<double> sklt_out, vel_out;
    //     double t1, t2, dt;
    //     double ts0;

    //     while (!isStopping())
    //     {
    //         double tnow = Time::now() - t0;

    //         // ---------- VELOCITY ----------
    //         // read events
    //         event_stats = input_events.readAll(false);
    //         if (event_stats.count == 0)
    //             continue;

    //         if(!started)
    //         {
    //             started = true;
    //             ts0 = event_stats.timestamp;
    //         } 

    //         // update images
    //         for (auto &v : input_events)
    //         {
    //             vis_image.at<cv::Vec3b>(v.y, v.x) = cv::Vec3b(255, 255, 255);
    //         }

    //         t1 = Time::now();
                       

    //         // only update velocity if the pose is initialised
    //         if (!state.poseIsInitialised())
    //             continue;

    //         pw_trip_velocity.updateSAE(input_events.begin(), input_events.end(), event_stats.timestamp-ts0); 
    //         skel_vel = pw_trip_velocity.query(state.query(), event_stats.timestamp-ts0, roiSize, 1);
            
            
    //         for (int j = 0; j < 13; j++) // (F) overload * to skeleton13
    //             skel_vel[j] = skel_vel[j] * scaler;
    //         state.setVelocity(skel_vel);
    //         state.updateFromVelocity(skel_vel, event_stats.timestamp);

    //         t2 = Time::now();
    //         dt = (t2 - t1) * 1e3;


    //         sklt_out.clear();
    //         vel_out.clear();
    //         for (int j = 0; j < 13; j++)
    //         {
    //             sklt_out.push_back(skeleton_detection[j].u);
    //             sklt_out.push_back(skeleton_detection[j].v);
    //             vel_out.push_back(skel_vel[j].u);
    //             vel_out.push_back(skel_vel[j].v);
    //         }
    //         ros_output.timestamp = tnow;
    //         ros_output.pose = sklt_out;
    //         ros_output.velocity = vel_out;
    //         // publish data
    //         ros_publisher.prepare() = ros_output;
    //         ros_publisher.write();
            
    //     }
    // }
};

int main(int argc, char *argv[])
{
    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.configure(argc, argv);

    /* create the module */
    APRIL_HPE instance;
    return instance.runModule(rf);
}
