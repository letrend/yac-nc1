#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <visp/vpV4l2Grabber.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpPose.h>
#include <visp3/core/vpImageTools.h>
#include "pose_helper.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <regex>
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace cv;
using namespace std;

#define LOW_RES 0
#define HIGH_RES 1

class Camera {
public:
bool initCamera(string lo_res_calib_file_path,string hi_res_calib_file_path);
bool grabFrame(Mat &img, int resolution=0);
bool grabFrame(Mat &img, Mat &img_raw, int resolution);
bool detectQRCode(Mat &img, vector<vpImagePoint> &qr_code_corners,
                  vpRowVector &qr_code_pos, vpQuaternionVector &qr_code_rot,
                  float &qr_code_size);

Size lo_res, hi_res;
Mat cameraMatrix_lo_res, cameraMatrix_hi_res, distCoeffs_lo_res, distCoeffs_hi_res;
int resolution = LOW_RES;
private:
void changeResolution(int resolution);
VideoCapture cap;
Mat frame, frame_undistorted;
mutex mux;
};
