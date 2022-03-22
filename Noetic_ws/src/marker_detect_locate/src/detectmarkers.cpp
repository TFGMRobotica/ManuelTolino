/* 

File: detectmarkers.cpp 

Author: Manuel Tolino Contreras

Decription: This file contains the code intended to generate a node which should
capture a gstreamer video input and detect ArUco markers. The detected
markers should be represented in real time in a windows superposed with
the video feed.

*/


#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <cstdlib>

namespace {
const char* about = "Detect ArUco marker images";
const char* keys  =
        "{d        |16    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
        "DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
        "DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
        "DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
        "DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{h        |false | Print help }"
        "{v        |<none>| Custom video source, otherwise '0' }"
        ;
}

int main(int argc, char **argv)
{
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (parser.get<bool>("h")) {
        parser.printMessage();
        return 0;
    }

    int dictionaryId = parser.get<int>("d");
    int wait_time = 10;
    cv::String videoInput = "0";
    cv::VideoCapture in_video;

    /*Parametros copiados de inet
    const cv::Mat  intrinsic_matrix = (cv::Mat_<float>(3, 3)
                               << 803.9233,  0,         286.5234,
                                  0,         807.6013,  245.9685,
                                  0,         0,         1);
                                  */
     const cv::Mat  intrinsic_matrix = (cv::Mat_<float>(3, 3)
                               << 530.8269276712998,  0.0,       320.5,
                                  0.0,       530.8269276712998,  240.5,
                                  0.0,       0.0,                  1.0);
                                                                  
    /* NO distorsion for the virtual camera*/
    const cv::Mat  distCoeffs = (cv::Mat_<float>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
    /* const cv::Mat  distCoeffs = (cv::Mat_<float>(5, 1) << 0.1431, -0.4943, 0, 0, 0); */


    const cv::Mat  arucodistCoeffs = (cv::Mat_<float>(1, 5) << 0, 0, 0, 0, 0);// La foto corregida se utiliza para la detección
    float fov_horiz = 1.0 ;
    float fov_vert = 1.0 ;

    /* The output parameters rvecs and tvecs are the rotation and translation vectors respectively, for each of the markers in markerCorners.*/
    /* The markerCorners parameter is the vector of marker corners returned by the detectMarkers() function.*/
    std::vector<cv::Vec3d> rvecs, tvecs; 
    
    if (parser.has("v")) {
        videoInput = parser.get<cv::String>("v");
        if (videoInput.empty()) {
            parser.printMessage();
            return 1;
        }
        char* end = nullptr;
        int source = static_cast<int>(std::strtol(videoInput.c_str(), &end, \
            10));
        if (!end || end == videoInput.c_str()) {
            in_video.open(videoInput); // url
        } else {
            in_video.open(source); // id
            in_video.set(cv::CAP_PROP_FPS, 30);
            in_video.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            in_video.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
            in_video.set(cv::CAP_PROP_SATURATION, 0);
        }
    } else {
        in_video.open("udpsrc port=5600 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1");
        //in_video.set(cv::CAP_PROP_FPS, 30);
        //in_video.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        //in_video.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
        //in_video.set(cv::CAP_PROP_SATURATION, 0);
        printf("OPENED");
    }

    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }

    if (!in_video.isOpened()) {
        std::cerr << "failed to open video input: " << videoInput << std::endl;
        return 1;
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    while (in_video.grab()) {
        cv::Mat image, image_copy;
        in_video.retrieve(image);
        image.copyTo(image_copy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        int res_horizontal = image_copy.size().width;
        int res_vertical = image_copy.size().height;

        /*  DEBUGGING PRINT of camera input total pixels */
        // printf("Width: %i  Height: %i\n",horizontal_res,vertical_res);   
        
        
        // If at least one marker detected
        if (ids.size() > 0)
            {
            /*Estimate pose function*/
            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, intrinsic_matrix, distCoeffs, rvecs, tvecs);
            cv::aruco::drawDetectedMarkers(image_copy, corners, ids);

            for (int i = 0; i < rvecs.size(); ++i) 
            {   /* Draw axis for each marker detected*/
                auto rvec = rvecs[i];
                auto tvec = tvecs[i];
                cv::aruco::drawAxis(image_copy, intrinsic_matrix, distCoeffs, rvec, tvec, 0.1);
                }
            int marker_index = 0;    
            auto selected_marker = corners[marker_index];
            auto corner1 = selected_marker[0]; // Top Left, small ref. red square
            auto corner2 = selected_marker[1]; // Clockwise or top right
            auto corner3 = selected_marker[2]; // Clockwise or bottom right
            auto corner4 = selected_marker[3]; // Clockwise or bottom left

            double x_sum = corner1.x + corner2.x + corner3.x + corner4.x ;
            double y_sum = corner1.y + corner2.y + corner3.y + corner4.y ;

            double x_avg = x_sum / 4;
            double y_avg = y_sum / 4; 

            double x_dev = (x_avg - res_horizontal * .5) * fov_horiz / res_horizontal;
            double y_dev = (y_avg - res_vertical * .5) * fov_vert / res_vertical;

            /*  DEBUGGING PRINT of Deviation    */

            printf("Xdev=%f Ydev=%f\n", x_dev, y_dev);

            /*  DEBUGGING PRINT of 
            printf("X1=%i X2=%i X3=%i X4=%i\nY1=%i Y2=%i Y3=%i Y4=%i\n", int(corner1.x), int(corner2.x), int(corner3.x), int(corner4.x),
                                                int(corner1.y), int(corner2.y), int(corner3.y), int(corner4.y));
            */
            };
           
        imshow("Detected markers", image_copy);
        char key = (char)cv::waitKey(wait_time);
        if (key == 27)
            break;
    }
    in_video.release();
    return 0;
}