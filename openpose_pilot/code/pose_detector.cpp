
#include <iostream>

#include <opencv2/opencv.hpp>

// Command-line user interface
// #define OPENPOSE_FLAGS_DISABLE_POSE
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>

#include <yarp/cv/Cv.h>
#include <yarp/os/all.h>
#include <yarp/sig/Image.h>

#include <unistd.h>


using namespace yarp::cv;
using namespace yarp::os;
using namespace yarp::sig;


// Custom flags
DEFINE_string(yarp_image_producer, "camera/out", "Yarp port name of the frame producer.");
DEFINE_bool(no_display, false, "Enable to disable the visual display.");


// This worker will just read and return all the jpg files in a directory

cv::Mat convertToCV(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    try
    {
        // User's displaying/saving/other processing here
        // datum.cvOutputData: rendered frame with pose or heatmaps
        // datum.poseKeypoints: Array<float> with the estimated pose
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // Display image
            const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
            if (!cvMat.empty())
            {
                return cvMat;
            }
            else
                op::opLog("Empty cv::Mat as output.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
        }
        else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }

}

void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    try
    {
        // User's displaying/saving/other processing here
        // datum.cvOutputData: rendered frame with pose or heatmaps
        // datum.poseKeypoints: Array<float> with the estimated pose
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // Display image
            const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
            if (!cvMat.empty())
            {
                cv::imshow("APRIL", cvMat);
                cv::waitKey(1);
            }
            else
                op::opLog("Empty cv::Mat as output.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
        }
        else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}


// void getImage(BufferedPort<ImageOf<PixelRgb>> inPort, cv::Mat &imgCV)
// {
//     ImageOf<PixelRgb>* imgYarp = inPort.read();
//     if (imgYarp == nullptr)
//     {
//         std::cout << "failed to read yarp image" << std::endl;
//         return;
//     }

//     imgCV = toCvMat(*imgYarp);
// }


void initPoseDetector(op::Wrapper &detector)
{
    op::opLog("Configuring OpenPose...", op::Priority::High);

    try
    {
        const auto poseMode = op::flagsToPoseMode(FLAGS_body);
        const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
        const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
        const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
        const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
        const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
        const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                      FLAGS_heatmaps_add_PAFs);
        const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
        const bool enableGoogleLogging = false;

        const op::WrapperStructPose wrapperStructPose{
            poseMode, netInputSize, 1., outputSize, keypointScaleMode, FLAGS_num_gpu,
            FLAGS_num_gpu_start, FLAGS_scale_number, (float)FLAGS_scale_gap,
            op::flagsToRenderMode(FLAGS_render_pose, multipleView), poseModel, !FLAGS_disable_blending,
            (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap, FLAGS_part_to_show, op::String(FLAGS_model_folder),
            heatMapTypes, heatMapScaleMode, FLAGS_part_candidates, (float)FLAGS_render_threshold,
            FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max, op::String(FLAGS_prototxt_path),
            op::String(FLAGS_caffemodel_path), (float)FLAGS_upsampling_ratio, enableGoogleLogging};
        detector.configure(wrapperStructPose);

        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            detector.disableMultiThreading();
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}


void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    try
    {
        // Example: How to use the pose keypoints
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // Alternative 1
            op::opLog("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);

            // // Alternative 2
            // op::opLog(datumsPtr->at(0)->poseKeypoints, op::Priority::High);

            // // Alternative 3
            // std::cout << datumsPtr->at(0)->poseKeypoints << std::endl;

            // // Alternative 4 - Accesing each element of the keypoints
            // op::opLog("\nKeypoints:", op::Priority::High);
            // const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
            // op::opLog("Person pose keypoints:", op::Priority::High);
            // for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
            // {
            //     op::opLog("Person " + std::to_string(person) + " (x, y, score):", op::Priority::High);
            //     for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
            //     {
            //         std::string valueToPrint;
            //         for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
            //             valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
            //         op::opLog(valueToPrint, op::Priority::High);
            //     }
            // }
            // op::opLog(" ", op::Priority::High);
        }
        else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}


void runPoseDetector(op::Wrapper &detector, cv::Mat &image)
{
    try
    {
        // const auto opTimer = op::getTimerInit();

        const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(image);
        auto datumProcessed = detector.emplaceAndPop(imageToProcess);
        if (datumProcessed != nullptr)
        {
            // printKeypoints(datumProcessed);
            if (!FLAGS_no_display)
                display(datumProcessed);
            image = convertToCV(datumProcessed);
        }
        else
            std::cout << "Image could not be processed" << std::endl;

        // Measuring total time
        // op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}


int main(int argc, char *argv[])
{
    std::cout << "parsing commands" << std::endl;

    // Parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::cout << "setting up yarp input port" << std::endl;

    // Set up YARP ports and connections
    Network yarp;
    BufferedPort<ImageOf<PixelRgb>> inPort;
    BufferedPort< ImageOf<PixelRgb> > outPort;
    bool ok = inPort.open("/pose_detector/img:i");
    if (!ok) {
        std::cout << "Failed to create input port" << std::endl;
        std::cout << "Maybe you need to start a nameserver (run 'yarpserver')" << std::endl;
        return 1;
    }

    if(!outPort.open("/pose_detector/img:o")) {
        yError() << "Failed to create output port";
        return 1;
    }
    
    std::cout << "connecting ports" << std::endl;
    yarp.connect(FLAGS_yarp_image_producer, inPort.getName());

    std::cout << "configuring detector" << std::endl;
    op::Wrapper poseDetector{op::ThreadManagerMode::Asynchronous};
    initPoseDetector(poseDetector);

    std::cout << "starting detector" << std::endl;
    poseDetector.start();

    while(true)
    {
        std::cout << "getting image" << std::endl;
        sleep(1);

        cv::Mat imgCV;
        ImageOf<PixelRgb>* imgYarp = inPort.read();
        if (imgYarp == nullptr)
        {
            std::cout << "failed to read yarp image" << std::endl;
        }

        imgCV = toCvMat(*imgYarp);

        // getImage(inPort, imgCV);

        std::cout << "running pose detector" << std::endl;

        runPoseDetector(poseDetector, imgCV);

        std::cout << "image processed" << std::endl;

        outPort.prepare().copy(yarp::cv::fromCvMat<PixelBgr>(imgCV));
        outPort.write();
    }

    return 0;
}
