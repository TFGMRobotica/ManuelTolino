#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <cstdlib>

namespace {

const char* about = "Deteccion de marcadores ArUco";

//Diccionarios ArUco a definir en línea de comandos

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
    /**Funciones de apertura y linea de comandos*/

    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (parser.get<bool>("h")) {
        parser.printMessage();
        return 0;
    }

    int dictionaryId = parser.get<int>("d");
    int wait_time = 10;
    cv::String videoInput = "0";

    cv::VideoCapture in_video; /* Objeto de captura de video OpenCV*/
    if (parser.has("v")) { /* Si se ha introducido la opcion -v en la linea de comandos*/
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
    } else { /* Si no introducimos nada - por ahora usamos esta*/
        /* descartar si no se usa */
        //in_video.open("udpsrc port=5600 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false ! appsink drop=1");
        /* descartar si no se usa */

        /* Se obtiene el vídeo de un streaming mediante gstreamer la configuración en bruto se introduce entre ""*/
        in_video.open("udpsrc port=5600 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1");
        //in_video.set(cv::CAP_PROP_FPS, 30);
        //in_video.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        //in_video.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
        //in_video.set(cv::CAP_PROP_SATURATION, 0);
        printf("Entrada de vídeo abierta");
    }

    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }

    if (!in_video.isOpened()) {
        std::cerr << "No se ha podido iniciar la recepción de vídeo: " << videoInput << std::endl;
        return 1;
    }

    /* Selección del diccionario ArUco */
    /* Si no se pasa por línea de comandos ¿cual es el por defecto? ToDo*/
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

/* Bucle principal */
    while (in_video.grab()) {
        /* Elementos matriz de OpenCV, donde se guardan los bits de imagen*/
        cv::Mat image, image_copy;
        /* Obtiene un frame*/
        in_video.retrieve(image);
        /* Se copian los bits a una mat auxiliar*/
        image.copyTo(image_copy);
        /* Vector donde se almacenan los ids de los marcadores detectados*/
        std::vector<int> ids;
        /* Vector de coordenadas 2D de las esquinas detectadas*/
        std::vector<std::vector<cv::Point2f>> corners;
        /* Función OpenCV de detección de los marcadores*/
        cv::aruco::detectMarkers(image, dictionary, corners, ids);
        
        /* Si al menos se ha detectado un marcador...*/
        if (ids.size() > 0)
        /* Se superpone el contorno del marcador detectado sobre el mismo en la imagen*/
            cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
        /* Se representa en pantalla la imagen obtenida*/
        imshow("Detected markers", image_copy);
        /* Función para salir del programa con la letra Q. Podría omitirse?*/
        char key = (char)cv::waitKey(wait_time);
        if (key == 27)
            break;
    }
    /*Si no se detecta vídeo, cerrar objeto de captura de vídeo*/
    in_video.release();
    return 0;
}