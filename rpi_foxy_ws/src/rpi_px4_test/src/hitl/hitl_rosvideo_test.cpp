#/*
File: sitl_landing_test.cpp
Author: Manuel Tolino Contreras
Description: Este programa permite al UAV aterrizar sobre el marcador con id Determinada usando IRLOCKREPORT. (Aproximacion bidimensional)
Se utiliza sensor de distancia para tener feedback de la altitud y seleccionar así segundo marcador mas pequeño al estar mas bajo.
*/

#include <cstdlib>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include <px4_msgs/msg/irlock_report.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <string>
#include <opencv2/core.hpp> 
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

// for compressing the image
//#include <image_transport/image_transport.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
// for compressing the image
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>
#include <sensor_msgs/image_encodings.hpp>
//#include <image_transport/image_transport.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <opencv2/imgproc/imgproc.hpp>


#include <chrono>
#include <iostream>

using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


/**************** OpenCV global variables *****************/
//cv_bridge::CvImage::toImageMsg()
 /* Video object */
 //cv::VideoCapture in_video; rpicamera
cv::VideoCapture in_video;

/**************** OpenCV parameters *****************/
cv::Mat image /*,image_copy*/;

/************ ROS2 Node ************/

class DebugVectAdvertiser : public rclcpp::Node
{
public:
	DebugVectAdvertiser() : Node("ros_camera_receiver") {


		/*image_sub =
			this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});*/
		image_sub =	this->create_subscription<sensor_msgs::msg::Image>(
           "image_raw",
           10,
           std::bind(&DebugVectAdvertiser::imageCallback, this));

	}
private:

	void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
		{
		try {
			cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
			cv::waitKey(3);
		} catch (cv_bridge::Exception & e) {
			auto logger = rclcpp::get_logger("my_subscriber");
			RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		}
		};

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;   
};


int main(int argc, char* argv[])
{
    // in_video.open(0); rpicamera
	
    //in_video.open("udpsrc port=5600 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1");
	// HITL:
	//in_video.open("-v udpsrc port=5600 caps ='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! videoconvert ! fpsdisplaysink sync=false");

	std::cout << "Video input received!" << std::endl;
	std::cout << "Starting ArUco autoland control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugVectAdvertiser>());

    //in_video.release();
    rclcpp::shutdown();
    return 0;
}