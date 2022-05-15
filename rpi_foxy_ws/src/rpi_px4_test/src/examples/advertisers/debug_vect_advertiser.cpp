#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <cstdlib>

using namespace std::chrono_literals;

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
			auto debug_vect = px4_msgs::msg::DebugVect();
			debug_vect.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
			std::string name = "test";
			std::copy(name.begin(), name.end(), debug_vect.name.begin());
			cv::String videoInput = "0";
    		//cv::VideoCapture in_video;
			cv::VideoCapture in_video(0); 

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
			//in_video.open("udpsrc port=5600 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1");
			if (!in_video.isOpened()) {
				std::cerr << "failed to open video input: " << videoInput << std::endl;
			}

			cv::Ptr<cv::aruco::Dictionary> dictionary =
			cv::aruco::getPredefinedDictionary( \
			cv::aruco::PREDEFINED_DICTIONARY_NAME(16));
    		//while (in_video.grab()) {
			//if (in_video.grab()) {
				//bool gotframe = in_video.grab();
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
					debug_vect.x = x_dev;
					debug_vect.y = y_dev;
					//imshow("Detected markers", image_copy);
					/*  DEBUGGING PRINT of 
					printf("X1=%i X2=%i X3=%i X4=%i\nY1=%i Y2=%i Y3=%i Y4=%i\n", int(corner1.x), int(corner2.x), int(corner3.x), int(corner4.x),
														int(corner1.y), int(corner2.y), int(corner3.y), int(corner4.y));
					*/
					debug_vect.z = 4.0;
					this->publisher_->publish(debug_vect);
					RCLCPP_INFO(this->get_logger(), "\033[97m Publishing debug_vect: time: %llu x: %f y: %f z: %f \033[0m",
					debug_vect.timestamp, debug_vect.x, debug_vect.y, debug_vect.z);
					//char key = (char)cv::waitKey(5);
        			//if (key == 27)
            		//	break;
            		//};
					//in_video.release();
				};
			//in_video.release();
			//debug_vect.x = x_dev;
			//debug_vect.y = y_dev;			
		};
		timer_ = this->create_wall_timer(1ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::DebugVect>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting the debug_vect advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DebugVectAdvertiser>());
	rclcpp::shutdown();
	return 0;
}
