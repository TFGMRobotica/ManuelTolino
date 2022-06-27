#/*

File: debug_vect_advertiser.cpp
Author: Manuel Tolino Contreras
Description:

*/

#include <chrono>
#include <iostream>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>

/**************** OpenCV global variables *****************/

 /* Video object  -1st device - rpi camera module*/
cv::VideoCapture in_video(0);
//cv::VideoWriter writer;


/**************** OpenCV parameters *****************/

cv::String videoInput = "0";

/* ArUco Dictionary ID*/
cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(16));
	/*namespace {
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
	}*/
float fov_horiz = 1.0 ;
float fov_vert = 1.0 ;

/* Camera intrinsic matrix */
const cv::Mat  intrinsic_matrix = (cv::Mat_<float>(3, 3)
                               << 530.8269276712998,  0.0,       320.5,
                                  0.0,       530.8269276712998,  240.5,
                                  0.0,       0.0,                  1.0);

/* Distortion*/

const cv::Mat  distCoeffs = (cv::Mat_<float>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
const cv::Mat  arucodistCoeffs = (cv::Mat_<float>(1, 5) << 0, 0, 0, 0, 0); // ToDelete?

/* Other parameters not necessary for now

in_video.open(source); // id
in_video.set(cv::CAP_PROP_FPS, 30);
in_video.set(cv::CAP_PROP_FRAME_WIDTH, 640);
in_video.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
in_video.set(cv::CAP_PROP_SATURATION, 0);
*/

/************  ************/
using namespace std::chrono_literals;

/************ ROS2 Node ************/
class DebugVectAdvertiser : public rclcpp::Node
{
public:
	DebugVectAdvertiser() : Node("debug_vect_advertiser") {

#ifdef ROS_DEFAULT_API
		publisher_ = this->create_publisher<px4_msgs::msg::DebugVect>("fmu/debug_vect/in", 10);
#else
		publisher_ = this->create_publisher<px4_msgs::msg::DebugVect>("fmu/debug_vect/in");

#endif
		auto timer_callback =
		[this]()->void {

            /* ROS2 */
			auto debug_vect = px4_msgs::msg::DebugVect();
			debug_vect.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
			std::string name = "test";
			std::copy(name.begin(), name.end(), debug_vect.name.begin());

            /* OpenCV script*/
            in_video.grab();
            cv::Mat image, image_copy;
            in_video.retrieve(image);
            image.copyTo(image_copy);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::detectMarkers(image, dictionary, corners, ids);

            int res_horizontal = image_copy.size().width;
            int res_vertical = image_copy.size().height;

            /*  DEBUGGING PRINT of camera input total pixels */
            // printf("Width: %i  Height: %i\n",horizontal_res,vertical_res);
            
            if (ids.size() > 0) // If at least one marker detected
                {
                /*Estimate pose function*/
	            /* The output parameters rvecs and tvecs are the rotation and translation vectors respectively, for each of the markers in markerCorners.*/
                /* The markerCorners parameter is the vector of marker corners returned by the detectMarkers() function.*/
                cv::aruco::estimatePoseSingleMarkers(corners, 0.05, intrinsic_matrix, distCoeffs, rvecs, tvecs);

                /* Draw the markers in the output picture - not necessary now*/
                /*cv::aruco::drawDetectedMarkers(image_copy, corners, ids);*/

                /* Draw axis for each marker detected - not necessary now */
                /*
                for (int i = 0; i < rvecs.size(); ++i)
                    {   
                    auto rvec = rvecs[i];
                    auto tvec = tvecs[i];
                    cv::aruco::drawAxis(image_copy, intrinsic_matrix, distCoeffs, rvec, tvec, 0.1);
                    }
                */
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
                // printf("Xdev=%f Ydev=%f\n", x_dev, y_dev);
                debug_vect.x = x_dev;
                debug_vect.y = y_dev;
                debug_vect.z = 4.0;
                /*  DEBUGGING PRINT of
                printf("X1=%i X2=%i X3=%i X4=%i\nY1=%i Y2=%i Y3=%i Y4=%i\n", int(corner1.x), int(corner2.x), int(corner3.x), int(corner4.x),
                                                    int(corner1.y), int(corner2.y), int(corner3.y), int(corner4.y));
                */
                };
            /* Image output for debugging - does not work here properly*/    
            //imshow("Detected markers", image_copy); 
            /* ROS 2 Node console output for debugging*/
			RCLCPP_INFO(this->get_logger(), "\033[97m Publishing debug_vect: time: %llu x: %f y: %f z: %f \033[0m",
                                debug_vect.timestamp, debug_vect.x, debug_vect.y, debug_vect.z);
			this->publisher_->publish(debug_vect);
		};
	timer_ = this->create_wall_timer(1ms, timer_callback);
	}
private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::DebugVect>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    in_video.open(0);
    /*
    writer.open("appsrc ! videoconvert ! x264enc noise-reduction=10000 tune=zerolatency byte-stream=true threads=4 ! mpegtsmux ! udpsink host=localhost port=9999"
                , 0, (double)30, cv::Size(640, 480), true);
    if (!writer.isOpened()) {
        printf("=ERR= can't create video writer\n");
        return -1;
    }*/

	std::cout << "Starting debug_vect advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugVectAdvertiser>());

    in_video.release();
    rclcpp::shutdown();
    return 0;
}
