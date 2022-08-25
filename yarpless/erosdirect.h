#pragma once

#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/base/events/event_cd.h>
#include <hpe-core/representations.h>
#include <opencv2/opencv.hpp>
#include <chrono>

class EROSdirect
{
public:
    Metavision::Camera cam;
    hpecore::surface eros;
    cv::Mat filter;
    double thresh;
    cv::Size res;

    bool start(double sensitivity, double filter_value)
    {
        if(sensitivity < 0.0 || sensitivity > 1.0)
        {
            std::cerr << "[ERR] sensitivity 0 < s < 1" << std::endl;
            return false;
        }

        try {
            cam = Metavision::Camera::from_first_available();
            Metavision::I_LL_Biases* bias_control = cam.biases().get_facility();  
            int diff_on  = (66 - 350) * sensitivity + 650 - 66;
            int diff_off = (66 + 200) * sensitivity + 100 - 66;
            bias_control->set("bias_diff_on", diff_on);
            bias_control->set("bias_diff_off", diff_off);
        } catch(const std::exception &e) {
            std::cerr << "[ERR] no camera :(" << std::endl;
            return false;
        }

        const Metavision::Geometry &geo = cam.geometry();
        std::cout << "[" << geo.width() << "x" << geo.height() << "]" << std::endl;

        res =  cv::Size(geo.width(), geo.height());
        eros.init(res.width, res.height, 7, 0.3);
        thresh = filter_value;
        filter = cv::Mat(res.width, res.height, CV_32F);

        cam.cd().add_callback([this](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
            this->erosUpdate(ev_begin, ev_end);
        });

        if (!cam.start()) {
            std::cerr << "[ERR] Could not start the camera" << std::endl;
            return false;
        }

        return true;
    }

    void stop()
    {
        cam.stop();
    }

    void erosUpdate(const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) 
    {
        using namespace std::chrono;

        double t = duration_cast<seconds>(system_clock::now().time_since_epoch()).count();


        
        // auto start = high_resolution_clock::now();
        // auto stop = high_resolution_clock::now();
        // auto duration = duration_cast<seconds>(start-stop);
        // std::chrono::system_clock::time_point t = duration_cast<seconds>(std::chrono::system_clock::now());
        //int t = 0;
        for(auto &v = ev_begin; v != ev_end; ++v) 
        {
            float &pt = filter.at<float>(v->y, v->x);
            if(t - pt > thresh) 
            {
                eros.EROSupdate(v->x, v->y);
                pt = t;
            }

        }
    }

};