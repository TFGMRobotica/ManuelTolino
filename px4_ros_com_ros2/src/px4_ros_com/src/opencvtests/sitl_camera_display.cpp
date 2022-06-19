/* 

File: sitl_camera_display.cpp 

Author: Manuel Tolino Contreras

Decription: This file contains the code intended to generate a node which should
capture a gstreamer video input and detect ArUco markers. The detected
markers should be represented in real time in a windows superposed with
the video feed.

*/


#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp> 
#include <opencv2/highgui.hpp> 
#include <iostream>
#include <cstdlib>
#include <string>

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
using namespace cv;

int MARKER_BIG_SIZEmm=409; 

// Trackbar call back function 
static void onChange(int pos, void* userInput); 
//Mouse callback 
static void onMouse(int event, int x, int y, int, void* userInput); 

// Trackbar call back function 
static void onChange(int pos, void* userData) { 
    if(pos <= 0) return; 
    // Aux variable for result 
    Mat imgBlur; 
    // Get the pointer input image     
    Mat* img= (Mat*)userData; 
    // Apply a blur filter 
    blur(*img, imgBlur, Size(pos, pos)); 
    // Show the result 
    imshow("Detected markers", imgBlur); 
}

//Mouse callback 
static void onMouse(int event, int x, int y, int, void* userInput) 
{ 
   if(event != EVENT_LBUTTONDOWN) 
           return; 
 
   // Get the pointer input image 
   Mat* img= (Mat*)userInput; 
    
   // Draw circle 
   circle(*img, Point(x, y), 10, Scalar(0,255,0), 3); 
 
   // Call on change to get blurred image 
   onChange(MARKER_BIG_SIZEmm, img); 
 
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

    int MARKER_BIG = 4;
    int MARKER_SMALL = 6;

    /*Parametros copiados de inet
    const cv::Mat  intrinsic_matrix = (cv::Mat_<float>(3, 3)
                               << 803.9233,  0,         286.5234,
                                  0,         807.6013,  245.9685,
                                  0,         0,         1);
                                  */
     /*const cv::Mat  intrinsic_matrix = (cv::Mat_<float>(3, 3)
                               << 530.8269276712998,  0.0,         320.5,
                                  0.0,       530.8269276712998,    240.5,
                                  0.0,       0.0,                  1.0);*/
    /*const cv::Mat  intrinsic_matrix = (cv::Mat_<float>(3, 3)
                               << 474.250810,  0.0,         403.777430,
                                  0.0,       474.152947,    399.072316,
                                  0.0,       0.0,                  1.0);      */
    const cv::Mat  intrinsic_matrix = (cv::Mat_<float>(3, 3)
                               << 600,  0.0,         400,
                                  0.0,       600,    400,
                                  0.0,       0.0,                  1.0);                         
    /* NO distorsion for the virtual camera*/
    const cv::Mat  distCoeffs = (cv::Mat_<float>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
    /* const cv::Mat  distCoeffs = (cv::Mat_<float>(5, 1) << 0.1431, -0.4943, 0, 0, 0); */


    const cv::Mat  arucodistCoeffs = (cv::Mat_<float>(1, 5) << 0, 0, 0, 0, 0);// La foto corregida se utiliza para la detección
    float fov_horiz = 1.570796327 ;
    float fov_vert = 1.570796327 ;

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
        printf("OPENED\n");
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
    
    Point text_position(80, 80);//Declaring the text position//
    int font_size = 1;//Declaring the font size//
    Scalar font_Color(0, 0, 0);//Declaring the color of the font//
    int font_weight = 2;//Declaring the font weight//
    
    // creating stringstream objects
    std::stringstream ss1;
    std::stringstream ss2;
    float dev_horiz = 0.0;
    float dev_vert = 0.0;

    while (in_video.grab()) {
        cv::Mat image, image_copy;
        in_video.retrieve(image);
        image.copyTo(image_copy);

        cv::namedWindow("Detected markers", WINDOW_NORMAL); 
        // create a trackbar 
        cv::createTrackbar("Big marker size (mm)", "Detected markers", &MARKER_BIG_SIZEmm, 500); 

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids_valid;
        std::vector<std::vector<cv::Point2f>> corners_valid;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);
        //landingmarker_big = ids[0];
        ids_valid = ids;
        corners_valid = corners;
        int res_horizontal = image_copy.size().width;
        int res_vertical = image_copy.size().height;

        /*  DEBUGGING PRINT of camera input total pixels */
        //printf("Width: %i  Height: %i\n",horizontal_res,vertical_res);   
        
        
        // If at least one marker detected
        if (ids.size() > 0)
            {
            //int ldgmkrbig_index = 0;
            int novalidmarkerflag = 0;

            for (int i = 0; i < int(ids.size()); i++){
                if (int(ids.at(i)) == MARKER_BIG){
                    //ldgmkrbig_index = i;
                    //printf("Big marker detected!\n");
                    ids_valid.resize(1);
                    corners_valid.resize(1);
                    //ids_valid[0]=ids.at(i);
                    //corners_valid[0]=corners.at(i);
                    ids_valid[0]=ids.at(i);
                    corners_valid[0]=corners.at(i);                    
                    float MKR_SIZE = MARKER_BIG_SIZEmm / 1000.0;
                    cv::aruco::estimatePoseSingleMarkers(corners_valid, MKR_SIZE, intrinsic_matrix, distCoeffs, rvecs, tvecs);
                    cv::aruco::drawDetectedMarkers(image_copy, corners_valid, ids_valid);
                    /* Draw axis for each marker detected*/
                    auto rvec = rvecs[0];
                    auto tvec = tvecs[0];
                    cv::aruco::drawAxis(image_copy, intrinsic_matrix, distCoeffs, rvec, tvec, 0.1);

                    int marker_index = 0;    
                    auto selected_marker = corners_valid[marker_index];
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
                    dev_horiz =  x_dev;
                    dev_vert =  y_dev;
                    novalidmarkerflag = 0;
                    /*
                } else if (ids.at(i) == MARKER_SMALL) {
                    cv::aruco::estimatePoseSingleMarkers(corners, 0.05, intrinsic_matrix, distCoeffs, rvecs, tvecs);
                    cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
                    auto rvec = rvecs[0];
                    auto tvec = tvecs[0];
                    cv::aruco::drawAxis(image_copy, intrinsic_matrix, distCoeffs, rvec, tvec, 0.1);
                    novalidmarkerflag = 0;*/
                } else {
                    novalidmarkerflag = 1;
                };
            };

            /*if (novalidmarkerflag == 1) {
                break;
            }*/

            //printf("MarkerID=%i\n", ids.at(0));
            /*Estimate pose function*/

            //cv::aruco::estimatePoseSingleMarkers(corners, 0.05, intrinsic_matrix, distCoeffs, rvecs, tvecs);
            //cv::aruco::drawDetectedMarkers(image_copy, corners, ids);

            

         
          
                

                /*  DEBUGGING PRINT of Deviation  */

                // printf("Xdev=%f Ydev=%f\n", x_dev, y_dev);

                /*  DEBUGGING PRINT of 
                printf("X1=%i X2=%i X3=%i X4=%i\nY1=%i Y2=%i Y3=%i Y4=%i\n", int(corner1.x), int(corner2.x), int(corner3.x), int(corner4.x),
                                                    int(corner1.y), int(corner2.y), int(corner3.y), int(corner4.y));
                */
                
            };
        // assigning the value of num_float to ss1
        
    
        // assigning the value of num_float to ss2
        

        // initializing two string variables with the values of ss1 and ss2
        // and converting it to string format with str() function
        std::string str1 = std::to_string(dev_horiz);
        //std::string str2 = ss2.str();
  
        putText(image_copy, str1, text_position,FONT_HERSHEY_COMPLEX, font_size,font_Color, font_weight);//Putting the text in the matrix//   
        imshow("Detected markers", image_copy);
        // setMouseCallback("Detected markers", onMouse, &image_copy); 
        // Call to onChange to init 
        //onChange(MARKER_BIG_SIZEmm, &image_copy);
        char key = (char)cv::waitKey(wait_time);
        if (key == 27)
            break;
    }
     
    cv::destroyWindow("Detected markers"); 
    in_video.release();
    return 0;
}