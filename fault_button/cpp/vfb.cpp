/* 
Author: Arren Glover
 */

#include <yarp/os/all.h>
#include <event-driven/core.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <string>
#include <fstream>
#include "../../april_msgs/yarp/rosmsg/april_msgs/Emergency.h"

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
    double k{0.01};       // seconds in window
    double p{0.01};      // detection rate
    double T{5e5};       // threshold events/second
    double initT{5e5};

    enum state_name{DISPLAY, FBR_DRAW, MONITOR, FINISHED};
    state_name state{DISPLAY};
    cv::Mat img, mask;

    typedef struct circle_parameters {
        cv::Point c;
        int       r;
        bool      d;
    } circle_parameters;

    circle_parameters c_fbr{{0,0},0,false};

    // strcts for 
    typedef struct rate_buffer {
        float time;
        std::deque<int> *buffer;
        int count;
    } rate_buffer;

    std::deque<int> trigger_buffer;

    rate_buffer trig_buffer{0.0, &trigger_buffer, 0};

    std::vector<int> rest_rates;
    std::vector<int> trigger_rates;

    // threshold adaptation parameters
    double buffer_time{1.0}; // time in seconds for buffer
    // the actual dimension of the buffer depends on the time
    // resolution(k) used fot the detection
    int buffer_size{100};

    // threshold adaptation variables
    bool autoThresh{true};
    bool button_pressed{false};

    // ros 
    yarp::os::Node* ros_node{nullptr};
    yarp::os::Publisher<yarp::rosmsg::april_msgs::Emergency> ros_publisher;

    bool loadCalibration(std::string calib_file_path)
    {
        yarp::os::ResourceFinder calib_reader;
        calib_reader.setDefault("from", calib_file_path);
        calib_reader.configure(0, 0);

        yarp::os::Bottle &ps = calib_reader.findGroup("VISUAL_FAULT_BUTTON_CALIB");
        if(ps.isNull()) return false;

        c_fbr.c.x = ps.find("x").asInt32();
        c_fbr.c.y = ps.find("y").asInt32();
        c_fbr.r   = ps.find("r").asInt32();
        T         = ps.find("T").asFloat64();

        yInfo() << "Loaded parameters";
        yInfo() << "[" << c_fbr.c.x << "," << c_fbr.c.y << "]@" << c_fbr.r << "pixels";
        yInfo() << "Threshold:" << T;

        return true;
    }

    bool saveCalibration(std::string calib_file_path)
    {
        std::ofstream writer;
        writer.open(calib_file_path, std::ios_base::trunc);
        if(!writer.is_open()) {
            yError() << "could not save file (ensure path exists?):" << calib_file_path;
            return false;
        }
        writer << "[VISUAL_FAULT_BUTTON_CALIB]" << std::endl;
        writer << std::endl;
        writer << "x " << c_fbr.c.x << std::endl;
        writer << "y " << c_fbr.c.y << std::endl;
        writer << "r " << c_fbr.r << std::endl;
        writer << "T " << std::fixed << std::setprecision(3) << T << std::endl;
        writer.flush();
        writer.close();
        return true;
    }

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

        //ros interface
        ros_node = new yarp::os::Node("/APRIL");
        if(!ros_publisher.topic("/sim/visualFaultButton/trigger"))
            yWarning() << "Could not open ROS publisher - messages will not be sent to ros";

        // =====READ PARAMETERS=====

        k = rf.check("k", Value(0.01)).asFloat64();
        p = rf.check("p", Value(0.01)).asFloat64();
        initT = rf.check("T", Value(5e5)).asFloat64();
        T = initT;
        buffer_time = rf.check("b", Value(1.0)).asFloat64();
        img = cv::Mat(img_size, CV_8UC3); img = 0;
        mask = cv::Mat(img_size, CV_8U); mask = 0;

        if(loadCalibration("calib_parameters.txt")) {
            state=MONITOR;
            autoThresh = false;
            makeMask();
        } else {
            state=DISPLAY;
        }

        // ===== TRY DEFAULT CONNECTIONS =====
        Network::connect("/file/ch0dvs:o",  getName("/AE:i"), "fast_tcp");
        Network::connect("/file/atis/AE:o", getName("/AE:i"), "fast_tcp");
        Network::connect("/atis3/AE:o",     getName("/AE:i"), "fast_tcp");

        cv::namedWindow(getName(),  cv::WINDOW_KEEPRATIO);
        cv::resizeWindow(getName(), {640, 480});

        // threshold adaptation parameters
        buffer_size = (int) buffer_time / k;

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
        saveCalibration("calib_parameters.txt");
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
        cv::putText(img_display, "Press SPACE to begin", cv::Point(img_size.width*0.05, img_size.height*0.95), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
        cv::putText(img_display, "Calibrating: Fault Button Position and Region", cv::Point(img_size.width*0.05, img_size.height*0.05), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});

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

        cv::putText(img_display, "Calibrating: Fault Button Position and Region", cv::Point(img_size.width*0.05, img_size.height*0.05), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
        cv::putText(img_display, "DRAW Fault Region:", cv::Point(img_size.width*0.05, img_size.height*0.80), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
        cv::putText(img_display, "1. click and hold on fault button position", cv::Point(img_size.width*0.05, img_size.height*0.85), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
        cv::putText(img_display, "2. drag to form the region size and release", cv::Point(img_size.width*0.05, img_size.height*0.9), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
        cv::putText(img_display, "3. repeat if needed. press space when finished", cv::Point(img_size.width*0.05, img_size.height*0.95), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});

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

        // change displayed events color to notify a trigger
        if(count > T * k && trig_buffer.count >= 0){
            cv::circle(img, c_fbr.c, c_fbr.r, {0, 0, 255}, 6);
            for(auto &v : input_events) {
                if(mask.at<char>(v.y, v.x)) {
                    img.at<cv::Vec3b>(v.y, v.x) = {0, 0, 200};
                }
            }
            if(!autoThresh) {
                static yarp::os::Stamp ys;
                ys.update();
                yarp::rosmsg::april_msgs::Emergency &rosmessage = ros_publisher.prepare();
                rosmessage.header.seq = ys.getCount();
                rosmessage.header.frame_id = "xmsg";
                rosmessage.header.stamp = ys.getTime();
                rosmessage.emergency_label = "visual_fault_button_triggered";
                ros_publisher.write();
            }
        }

        // show
        if(autoThresh) {
            cv::putText(img, "Calibrating Threshold", cv::Point(img_size.width*0.05, img_size.height*0.05), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
            cv::putText(img, "Attention: press space each time the fault button is pressed", cv::Point(img_size.width*0.05, img_size.height*0.9), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
            cv::putText(img, "press (a) to use current threshold | press (r) to reset threshold", cv::Point(img_size.width*0.05, img_size.height*0.95), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
        } else {
            cv::putText(img, "Monitoring Visual Fault Button", cv::Point(img_size.width*0.05, img_size.height*0.05), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
            cv::putText(img, "ROS emergency message enabled", cv::Point(img_size.width*0.05, img_size.height*0.9), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
            cv::putText(img, "press (a) to enter threshold calibration mode", cv::Point(img_size.width*0.05, img_size.height*0.95), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
        }
        cv::putText(img, "Current rate: " + std::to_string(count), cv::Point(img_size.width*0.05, img_size.height*0.10), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
        cv::putText(img, "Current threshold: " + std::to_string(T * k), cv::Point(img_size.width*0.05, img_size.height*0.15), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
        //cv::putText(img, "count: " + std::to_string(trig_buffer.count), cv::Point(img_size.width*0.05, img_size.height*0.15), cv::FONT_HERSHEY_PLAIN, 1.0, {255, 255, 255});
        cv::imshow(getName(), img);
        char c = cv::waitKey(1);
        // handle button presses
        if(c == '\e')
            state = FINISHED;
        else if(c == ' ') {
            // space represent a button press
            button_pressed = true;
        }
        else if(c == 'a')
        {   
            // disable / enabele automatic threshold
            autoThresh = !autoThresh;
        } else if(c == 'r') {
            if(autoThresh) {
                T = initT;
                rest_rates.clear();
                trigger_rates.clear();
                state=DISPLAY;
            }
        }

        if(autoThresh)
            thresholdCheck(count);
    }

    void thresholdCheck(int count)
    {    
        bool updated_lists = false; // if a list was updated

        // count is the detected event rate in the roi
        int removed_rate = updateRateBuffer(&trig_buffer, count);

        // add removed rate to rest rates list
        if(removed_rate > T * k) // current threshold
        {   
            rest_rates.push_back(removed_rate);
            updated_lists = true;
        }

        if(button_pressed){
            // add all rates to trigger rates list (or part of them)
            // remove all the rates from the buffer

            std::vector<int> tmp_buffer(100, -1); // TODO less than 100 rates
            std::copy(trig_buffer.buffer->cbegin(), trig_buffer.buffer->cend(), tmp_buffer.begin());
            std::sort(tmp_buffer.begin(), tmp_buffer.end());
            // add only the highest quartile of the trigger rates, this gives more accurate results
            for(int i=(int)0.75*buffer_size; i<buffer_size; i++){
                if(tmp_buffer[i] > 0)
                    trigger_rates.push_back(tmp_buffer[i]);
            }

            //empty the trig_buffer
            trig_buffer.buffer->clear();
            // TODO make configurable
            trig_buffer.count = - 3 * buffer_size;

            button_pressed = false;
            updated_lists = true;
        }

        // if either of the rest or trigger rates list was updated, 
        // update the threshold
        if(updated_lists && autoThresh)
            updateThreshold();
    }

    int updateRateBuffer(rate_buffer *rate_buffer, int count)
    {   
        rate_buffer->count ++;
        if(rate_buffer->count >= 0)
            rate_buffer->buffer->push_back(count);
    
        int head = -1;
        if(rate_buffer->count > buffer_size)
        {   
            head = rate_buffer->buffer->front();
            rate_buffer->buffer->pop_front();
            rate_buffer->count --;
        }

        return head;
    }

    void updateThreshold()
    {   
        std::sort(trigger_rates.begin(), trigger_rates.end());
        std::sort(rest_rates.begin(), rest_rates.end());
        
        float min_loss = std::numeric_limits<float>::infinity();
        float min_val = -1.0;
        float max_search = 100000, min_search=100;

        if(!trigger_rates.empty())
        {
            max_search = trigger_rates.back();
        }
        if(!rest_rates.empty())
        {
            min_search = rest_rates.front();
        }
        
        // 100 is the hardcoded number of thresholds to test
        // TODO make configurable
        // more values should give better result at the cost of longer computation
        float search_step = max_search / 100;
        
        for(float i=min_search; i<max_search; i+=search_step){
            float loss = thresholdLoss(i);
            if(loss < min_loss)
            {
                min_loss = loss;
                min_val = i;
            }
        }
        if(min_val > 0.0)
            T = min_val / k;

        min_loss = 0;

    }

    float thresholdLoss(int threshold, float w=0.5)
    {   
        int n_missed = 0, n_false = 0;
        float loss = 0.0;

        if(!trigger_rates.empty())
        {
            auto upper_t = std::upper_bound(trigger_rates.begin(), trigger_rates.end(), threshold);
            n_missed = std::distance(trigger_rates.begin(), upper_t);
        }

        if(!rest_rates.empty())
        {
            auto upper_r = std::upper_bound(rest_rates.begin(), rest_rates.end(), threshold);
            n_false = std::distance(upper_r, rest_rates.end());
        }
         
        if(!trigger_rates.empty()){
            loss += ((float)n_missed / (float)trigger_rates.size()) * (1.0 - w);
            loss += ((float)threshold / (float)trigger_rates.back()) * 0.05;
        }
        if(! rest_rates.empty()){
            loss += ((float)n_false / (float)rest_rates.size()) * w;
        }

        return loss;
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

    /* create the module */
    vbfApp instance;
    return instance.runModule(rf);
}