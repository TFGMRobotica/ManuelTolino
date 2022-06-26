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

#include <chrono>
#include <iostream>

using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/**************** OpenCV global variables *****************/

 /* Video object */
 //cv::VideoCapture in_video; rpicamera
cv::VideoCapture in_video;

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
float fov_horiz = 1.570796327 ;
float fov_vert = 1.570796327 ;

/* Camera intrinsic matrix */
const cv::Mat  intrinsic_matrix = (cv::Mat_<float>(3, 3)
                               << 600,  0.0,         400,
                                  0.0,       600,    400,
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


/************ ROS2 Node ************/

auto irlock_data = px4_msgs::msg::IrlockReport();
auto irlock_msg = px4_msgs::msg::IrlockReport();
int navstate = -1;
int LANDING_MARKER_ACTIVE = 4;
float SWITCH_AGL_ALT = 4;
float SWITCH_AGL_MARGIN = 1.5 ;
float altitude_agl;
int distance_quality;



class DebugVectAdvertiser : public rclcpp::Node
{
public:
	DebugVectAdvertiser() : Node("marker_landing_guidance") {

#ifdef ROS_DEFAULT_API
		publisher_ = this->create_publisher<px4_msgs::msg::IrlockReport>("fmu/irlock_report/in", 10);

#else
		publisher_ = this->create_publisher<px4_msgs::msg::IrlockReport>("fmu/irlock_report/in");

#endif
		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});
		vehiclestatus_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleStatus>("fmu/vehicle_status/out", 10,
				[this](const px4_msgs::msg::VehicleStatus::UniquePtr navstatmsg) {
					//std::cout << "ts: "          << navstatmsg->timestamp    << std::endl;
					//std::cout << "CONTROL MODE: " << navstatmsg->nav_state << std::endl;
					/*RCLCPP_INFO(this->get_logger(), "\033[97m Control mode: %u \033[0m",
                                navstatmsg->nav_state);*/
					navstate = navstatmsg->nav_state;
				});
		distancesensor_sub_ =
			this->create_subscription<px4_msgs::msg::DistanceSensor>("fmu/distance_sensor/out", 10,
				[this](const px4_msgs::msg::DistanceSensor::UniquePtr sensordistmsg) {
					//std::cout << "ts: "          << navstatmsg->timestamp    << std::endl;
					//std::cout << "CONTROL MODE: " << navstatmsg->nav_state << std::endl;
					/*RCLCPP_INFO(this->get_logger(), "\033[97m Altitude AGL: %f \033[0m",
                                sensordistmsg->current_distance);*/
					/*RCLCPP_INFO(this->get_logger(), "\033[97m OF Quality: %u \033[0m",
                                sensordistmsg->signal_quality);*/
					altitude_agl = sensordistmsg->current_distance;
					distance_quality = sensordistmsg->signal_quality;
				});
		
		auto timer_callback = [this]()->void {
            /* OpenCV script*/

			cv::Mat image /*,image_copy*/;
			std::vector<int> ids;
			std::vector<std::vector<cv::Point2f>> corners;
        	std::vector<int> ids_valid;
        	std::vector<std::vector<cv::Point2f>> corners_valid;
			std::vector<cv::Vec3d> rvecs, tvecs;
			cv::Mat image_copy(1200, 1200, CV_8UC3, cv::Scalar(0, 0, 0));

			in_video.grab();
			in_video.retrieve(image);
			image.copyTo(image_copy);
			cv::aruco::detectMarkers(image, dictionary, corners, ids);
			cv::aruco::drawDetectedMarkers(image_copy, corners, ids);

			if (navstate == 20 /*&& in_video.isOpened()*/){
				/* MOVED OUTSIDE FOR DEBUGGING
				in_video.grab();
				in_video.retrieve(image);
				image.copyTo(image_copy);
				cv::aruco::detectMarkers(image, dictionary, corners, ids);
				cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
				//imshow("cam", image_copy); 
				*/
				ids_valid = ids;
        		corners_valid = corners;
			};

			if (altitude_agl < SWITCH_AGL_ALT) {
				LANDING_MARKER_ACTIVE = 6;
			} else {
				LANDING_MARKER_ACTIVE = 4;
			};
            int res_horizontal = image_copy.size().width;
            int res_vertical = image_copy.size().height;

            /*  DEBUGGING PRINT of camera input total pixels */
            // printf("Width: %i  Height: %i\n",horizontal_res,vertical_res);
            
            if (ids.size() > 0 && navstate == 20) // If at least one marker detected
                {
				for (int i = 0; i < int(ids.size()); i++){
					if (int(ids.at(i)) == LANDING_MARKER_ACTIVE){
						ids_valid.resize(1);
						corners_valid.resize(1);
						ids_valid[0]=ids.at(i);
						corners_valid[0]=corners.at(i);
						//cv::aruco::estimatePoseSingleMarkers(corners_valid, 0.05, intrinsic_matrix, distCoeffs, rvecs, tvecs);
					};
				};

                int marker_index = 0;
                //auto selected_marker = corners[marker_index];
				auto selected_marker = corners_valid[marker_index];
                auto corner1 = selected_marker[0]; // Top Left, small ref. red square
                auto corner2 = selected_marker[1]; // Clockwise or top right
                auto corner3 = selected_marker[2]; // Clockwise or bottom right
                auto corner4 = selected_marker[3]; // Clockwise or bottom left

                double x_sum = corner1.x + corner2.x + corner3.x + corner4.x ;
                double y_sum = corner1.y + corner2.y + corner3.y + corner4.y ;

                double x_avg = x_sum / 4;
                double y_avg = y_sum / 4;

				Scalar hline_Color(0, 255, 0);
				Point hpt1(0, y_avg);
				Point hpt2(res_horizontal, y_avg);
				line(image_copy,hpt1,hpt2,hline_Color,1);

				Scalar vline_Color(0, 255, 0);
				Point vpt1(x_avg, 0);
				Point vpt2(x_avg, res_vertical);
				line(image_copy,vpt1,vpt2,vline_Color,1);



                double x_dev = (x_avg - res_horizontal * .5) * fov_horiz / res_horizontal;
                double y_dev = (y_avg - res_vertical * .5) * fov_vert / res_vertical;

				// =======  Dynaamic Text overlay  =========== //

				double font_size = 0.5;//Declaring the font size//
				Scalar font_Color(0, 255, 0);//Declaring the color of the font//
				int font_weight = 1;//Declaring the font weight//
				Point text_devx_position((x_avg + 10), 40 );//Declaring the text position//
				Point text_devy_position(40, (y_avg - 10));//Declaring the text position//
				std::string str1 = " Deviation X: ";
				std::string str2 = std::to_string(irlock_msg.pos_x);
				std::string str3 = " Deviation Y: ";
				std::string str4 = std::to_string(irlock_msg.pos_y);
				std::string overlaytext_devx = str1 + str2;
				std::string overlaytext_devy = str3 + str4;
				putText(image_copy, overlaytext_devx, text_devx_position,FONT_HERSHEY_SIMPLEX, font_size,font_Color, font_weight);
				putText(image_copy, overlaytext_devy, text_devy_position,FONT_HERSHEY_SIMPLEX, font_size,font_Color, font_weight);

				// ============================================ //

                /*  DEBUGGING PRINT of Deviation    */
                // printf("Xdev=%f Ydev=%f\n", x_dev, y_dev);
				irlock_data.timestamp = timestamp_.load();
                irlock_data.pos_x = x_dev;
                irlock_data.pos_y = y_dev;
                irlock_data.signature = 1;
                irlock_data.size_x = 0;
                irlock_data.size_y = 0;

				irlock_msg = irlock_data;
				this->publisher_->publish(irlock_msg);
				/* ROS 2 Node console output for debugging*/
				RCLCPP_INFO(this->get_logger(), "\033[97m TargetDev: x: %f y: %f ID: %i ALT: %f \033[0m",
                                irlock_msg.pos_x, irlock_msg.pos_y, LANDING_MARKER_ACTIVE, altitude_agl);
			    };

				// =======  Static Text overlay  =========== //

				Point text2_position(40, 120);//Declaring the text position//
				Point text3_position(40, 160);//Declaring the text position//

				double font_size = 0.7;//Declaring the font size//
				Scalar font_Color(0, 255, 0);//Declaring the color of the font//
				int font_weight = 2;//Declaring the font weight//


				
				std::string str7 = "Altitude: ";
				std::string str8 = std::to_string(altitude_agl);
				std::string overlaytext_altitude = str7 + str8;
				putText(image_copy, overlaytext_altitude, text3_position,FONT_HERSHEY_COMPLEX, font_size,font_Color, font_weight);
				
				std::string str5 = "Nav mode: ";
				std::string str6 = std::to_string(navstate);
				std::string overlaytext_navmode = str5 + str6;
				putText(image_copy, overlaytext_navmode, text2_position,FONT_HERSHEY_COMPLEX, font_size,font_Color, font_weight);

				// ============================================ //

                cv::imshow("Detected markers", image_copy); 
				cv::waitKey(2);

		};

	timer_ = this->create_wall_timer(1ms, timer_callback);
	}
private:




	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::IrlockReport>::SharedPtr publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehiclestatus_sub_;
	rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr distancesensor_sub_;    
    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	//DebugVectAdvertiser::interface(float devX, float devY, float alt, int navmode){
};


int main(int argc, char* argv[])
{
    // in_video.open(0); rpicamera
	
    in_video.open("udpsrc port=5601 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1");
	std::cout << "Video input received!" << std::endl;
	std::cout << "Starting ArUco autoland control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugVectAdvertiser>());

    in_video.release();
    rclcpp::shutdown();
    return 0;
}