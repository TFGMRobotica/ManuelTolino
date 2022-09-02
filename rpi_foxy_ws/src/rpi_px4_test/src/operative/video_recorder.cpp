#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
using namespace cv;
/// For the Raspberry Pi 64-bit Buster OS

std::string GetCurrentTimeForFileName()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%F_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '-');
    return s;
}
auto videofilename = GetCurrentTimeForFileName();
/*
std::string gstreamer_pipeline(int device, int capture_width, int capture_height, int framerate, int display_width, int display_height) {
    return
            " v4l2src device=/dev/video"+ std::to_string(device) + " !"
            " video/x-raw,"
            " width=(int)" + std::to_string(capture_width) + ","
            " height=(int)" + std::to_string(capture_height) + ","
            " framerate=(fraction)" + std::to_string(framerate) +"/1 !"
            " videoconvert ! videoscale !"
            " video/x-raw,"
            " width=(int)" + std::to_string(display_width) + ","
            " height=(int)" + std::to_string(display_height) + " ! appsink";
}
*/

std::string gstreamer_pipeline(int device, int capture_width, int capture_height, int framerate, int display_width, int display_height) {
    return
            " v4l2src device=/dev/video"+ std::to_string(device) + " !"
            " video/x-raw,"
            " width=(int)" + std::to_string(capture_width) + ","
            " height=(int)" + std::to_string(capture_height) + ","
            " framerate=(fraction)" + std::to_string(framerate) +"/1 !"
            " videoconvert ! videoscale !"
            " video/x-raw,"
            " width=(int)" + std::to_string(display_width) + ","
            " height=(int)" + std::to_string(display_height) + " ! appsink";
}



std::string gstreamer_writerpipeline(int bitrate, std::string videofilename) {
    return
	// videorate ! video/x-raw, width=640, height=380, framerate=30/1 !
	//" queue ! x264enc speed-preset=ultrafast tune=zerolatency "
            " appsrc ! videorate ! videoconvert !"
            " queue ! x264enc speed-preset=superfast tune=zerolatency"
            " bitrate=" + std::to_string(bitrate) + " !"
            " queue ! matroskamux !"
            " queue ! filesink location=/home/pi/tfm/rpi_foxy_ws/src/rpi_px4_test/src/operative"
            "/" + videofilename + ".mkv"
            " sync=true";
}

int main()
{
    float f;
    float FPS[16];
    int   i,Fcnt=0;
    //some timing
    std::chrono::steady_clock::time_point Tbegin, Tend;
    //pipeline parameters
    int device = 0;                 //0=raspicam 1=usb webcam (when both are connected)
    int capture_width = 640 ;
    int capture_height = 480 ;
    int framerate = 30 ;
    int display_width = 640 ;
    int display_height = 480 ;
    int bitrate = 1024;
    
    //reset frame average
    for(i=0;i<16;i++) FPS[i]=0.0;

    std::string writerpipeline = gstreamer_writerpipeline(bitrate, videofilename);
    std::string pipeline = gstreamer_pipeline(device,
                                              capture_width, capture_height, framerate,
                                              display_width, display_height);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n\n\n";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if(!cap.isOpened()) {
        std::cout<<"Failed to open camera."<<std::endl;
        return (-1);
    }
    
    cv::VideoWriter writer;
    writer.open( writerpipeline, 0, (double)30, cv::Size(640, 480), true);
	std::cout << "Using writer pipeline: \n\t" << writerpipeline << "\n\n\n";
    if (!writer.isOpened()) {
        printf("=ERR= can't create video writer\n");
        return -1;
    } 
    
    cv::namedWindow("Camera feed", cv::WINDOW_AUTOSIZE);
    cv::VideoWriter writer(videofilename+".avi",cv::VideoWriter::fourcc('M','J','P','G'),30, Size(640,480),true);
    cv::Mat frame;
    cv::Mat image;
    int key;
    double dt;
    dt = 0.0;
    std::cout << "Hit ESC to exit" << "\n" ;
    while(true)
    {
    	if (!cap.read(frame)) {
            std::cout<<"Capture read error"<<std::endl;
            break;
        }
        if (frame.empty()) break;

        //calculate frame rate
        Tend = std::chrono::steady_clock::now();
        f = std::chrono::duration_cast<std::chrono::milliseconds> (Tend - Tbegin).count();
        Tbegin = Tend;
        if(f>0.0) FPS[((Fcnt++)&0x0F)]=1000.0/f;
        for(f=0.0, i=0;i<16;i++){ f+=FPS[i]; }
        //cv::putText(frame, cv::format("FPS %0.2f", f/16),cv::Point(10,20),cv::FONT_HERSHEY_SIMPLEX,0.6, cv::Scalar(0, 0, 255));
        printf("FPS %0.2f\n", f/16);
        //show frame
        //cv::cvtColor(frame,image,cv::COLOR_RGB2BGR);
        writer << frame;
        cv::imshow("Camera feed",frame);
        char esc = cv::waitKey(5);
        if(esc == 27) break;

        //key = cv::waitKey( 5 );
    }

    cap.release();
    cv::destroyAllWindows() ;
    return 0;
}