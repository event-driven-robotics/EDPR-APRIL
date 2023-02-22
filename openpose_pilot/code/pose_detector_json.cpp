
#include <experimental/filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>

// Command-line user interface
// #define OPENPOSE_FLAGS_DISABLE_POSE
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>

#include <unistd.h>


namespace fs = std::experimental::filesystem;


// Custom flags
DEFINE_bool(no_display, false, "Enable to disable the visual display.");

DEFINE_string(input_folder, "", "Folder containing image frames");
DEFINE_string(output_folder, "", "Folder for storing pose files");


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

        const op::WrapperStructOutput wrapperStructOutput{
            FLAGS_cli_verbose, op::String(FLAGS_write_keypoint), op::stringToDataFormat(FLAGS_write_keypoint_format),
            op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json), FLAGS_write_coco_json_variants,
            FLAGS_write_coco_json_variant, op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
            op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
            op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
            op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
            op::String(FLAGS_udp_port)};
        detector.configure(wrapperStructOutput);

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
//            // Alternative 1
//            op::opLog("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);

            // // Alternative 2
            // op::opLog(datumsPtr->at(0)->poseKeypoints, op::Priority::High);

            // // Alternative 3
            // std::cout << datumsPtr->at(0)->poseKeypoints << std::endl;

             // Alternative 4 - Accesing each element of the keypoints
             op::opLog("\nKeypoints:", op::Priority::High);
             const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
             op::opLog("Person pose keypoints:", op::Priority::High);
             for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
             {
                 op::opLog("Person " + std::to_string(person) + " (x, y, score):", op::Priority::High);
                 for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
                 {
                     std::string valueToPrint;
                     for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
                         valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
                     op::opLog(valueToPrint, op::Priority::High);
                 }
             }
             op::opLog(" ", op::Priority::High);
        }
        else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}


//void runPoseDetector(op::Wrapper &detector, cv::Mat &image, std::string file_name, auto &datumsPtr)
void runPoseDetector(op::Wrapper &detector, cv::Mat &image, std::string file_name)
{
    try
    {
        const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(image);

        auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
        datumsPtr->emplace_back();
        auto& datumPtr = datumsPtr->at(0);
        datumPtr = std::make_shared<op::Datum>();

        datumPtr->cvInputData = imageToProcess;
        datumPtr->name = file_name;
        detector.emplaceAndPop(datumsPtr);
//        datumsPtr = detector.emplaceAndPop(imageToProcess);
        if (datumsPtr != nullptr)
        {
            // printKeypoints(datumProcessed);
            if (!FLAGS_no_display)
                display(datumsPtr);
            image = convertToCV(datumsPtr);
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

    // parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::cout << "configuring detector" << std::endl;
    op::Wrapper poseDetector{op::ThreadManagerMode::Asynchronous};
    initPoseDetector(poseDetector);

    std::cout << "starting detector" << std::endl;
    poseDetector.start();

    // read input folder
    // for every image frame
    for (const auto & entry : fs::directory_iterator(FLAGS_input_folder))
    {
        std::cout << "reading file" << std::endl;
        std::cout << entry.path() << std::endl;

        if (entry.path().extension() != ".png")
            continue;

        // convert grayscale to rgb image
        std::cout << "converting grayscale image to rgb format" << std::endl;
        cv::Mat imgCV = cv::imread(entry.path().string(), cv::IMREAD_ANYDEPTH);
        cv::Mat convertedSrc(imgCV.rows, imgCV.cols, CV_8UC3, cv::Scalar(0,0,255));
        if (imgCV.type() != convertedSrc.type())
            cvtColor(imgCV, convertedSrc, CV_GRAY2RGB);
        else
            imgCV.copyTo(convertedSrc);

        std::cout << "running pose detector" << std::endl;
//        std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datumProcessed;
//        runPoseDetector(poseDetector, convertedSrc, entry.path().stem().string(), datumProcessed);
        runPoseDetector(poseDetector, convertedSrc, entry.path().stem().string());

        // read output path from arguments

        // save image
        fs::path output_path_img = FLAGS_output_folder;
        output_path_img /= entry.path().filename();
        cv::imwrite(output_path_img.string(), convertedSrc);

//        // save pose
//        fs::path output_path_pose = FLAGS_output_folder;
//        output_path_pose /= entry.path().stem().string() + ".json";
//        printKeypoints(datumProcessed);

        std::cout << "image processed" << std::endl;
    }

    return 0;
}
