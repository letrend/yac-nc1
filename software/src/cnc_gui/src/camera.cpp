#include "camera.hpp"


bool Camera::initCamera(string lo_res_calib_file_path, string hi_res_calib_file_path, int deviceID){
        ROS_INFO("initializing camera with id %d",deviceID);
        FileStorage fs_lo, fs_hi;
        fs_lo.open(lo_res_calib_file_path, FileStorage::READ);
        fs_hi.open(hi_res_calib_file_path, FileStorage::READ);
        if (!fs_lo.isOpened()) {
                ROS_ERROR_STREAM("Failed to open " << lo_res_calib_file_path);
                return false;
        }
        if (!fs_hi.isOpened()) {
                ROS_ERROR_STREAM("Failed to open " << hi_res_calib_file_path);
                return false;
        }
        fs_lo["camera_matrix"] >> cameraMatrix_lo_res;
        fs_lo["distortion_coefficients"] >> distCoeffs_lo_res;
        fs_hi["camera_matrix"] >> cameraMatrix_hi_res;
        fs_hi["distortion_coefficients"] >> distCoeffs_hi_res;
        fs_lo["image_width"] >> lo_res.width;
        fs_lo["image_height"] >> lo_res.height;
        fs_hi["image_width"] >> hi_res.width;
        fs_hi["image_height"] >> hi_res.height;
        ROS_INFO("low resolution width %d height %d",lo_res.width, lo_res.height);
        ROS_INFO_STREAM("cameraMatrix: " << endl << cameraMatrix_lo_res);
        ROS_INFO_STREAM("distCoeffs: " << endl << distCoeffs_lo_res);
        ROS_INFO("high resolution width %d height %d",hi_res.width, hi_res.height);
        ROS_INFO_STREAM("cameraMatrix: " << cameraMatrix_hi_res);
        ROS_INFO_STREAM("distCoeffs: " << endl << distCoeffs_hi_res);
        try {
                // 0 = open default camera
                int apiID = cv::CAP_ANY;                 // 0 = autodetect default API
                // open selected camera using selected API
                cap.open(deviceID, apiID);
                changeResolution(LOW_RES);

                while (!cap.isOpened()) {                 // check if we succeeded
                        ROS_INFO_ONCE("waiting for camera to become available");
                        continue;
                }
        }
        catch(cv::Exception& e) {
                ROS_ERROR_STREAM_THROTTLE(5,e.what());
        }
}


bool Camera::grabFrame(Mat &img, int res){
        lock_guard<mutex> lock(mux);
        try {
                if(res!=resolution) {
                        changeResolution(res);
                }
                ros::Time t0 = ros::Time::now();
                do {
                        cap >>frame;
                        if((ros::Time::now()-t0).toSec()>5) {
                                ROS_ERROR("grabbing frame timed out");
                                return false;
                        }
                        // ROS_INFO_THROTTLE(1,"reading frame");
                } while(frame.empty());
                if(res==LOW_RES)
                        undistort(frame, frame_undistorted, cameraMatrix_lo_res, distCoeffs_lo_res);
                else if(res==HIGH_RES)
                        undistort(frame, frame_undistorted, cameraMatrix_hi_res, distCoeffs_hi_res);
                rotate(frame_undistorted, img, cv::ROTATE_90_COUNTERCLOCKWISE);
                return true;
        }catch(cv::Exception& e) {
                ROS_ERROR_STREAM_THROTTLE(5,e.what());
                return false;
        }
}

bool Camera::grabFrame(Mat &img, Mat &img_raw, int res){
        lock_guard<mutex> lock(mux);
        if(res!=resolution) {
                changeResolution(res);
        }
        ros::Time t0 = ros::Time::now();
        do {
                cap >> img_raw;
                if((ros::Time::now()-t0).toSec()>5) {
                        ROS_ERROR("grabbing frame timed out");
                        return false;
                }
                // ROS_INFO_THROTTLE(1,"reading frame");
        } while(img_raw.empty());
        if(res==LOW_RES)
                undistort(img_raw, frame_undistorted, cameraMatrix_lo_res, distCoeffs_lo_res);
        else if(res==HIGH_RES)
                undistort(img_raw, frame_undistorted, cameraMatrix_hi_res, distCoeffs_hi_res);
        rotate(frame_undistorted, img, cv::ROTATE_90_COUNTERCLOCKWISE);
        return true;
}

bool Camera::detectQRCode(Mat &img, vector<vpImagePoint> &qr_code_corners,
                          vpRowVector &qr_code_pos, vpQuaternionVector &qr_code_rot,
                          float &qr_code_size){
        vpCameraParameters cam(img.cols, img.rows, img.cols / 2, img.rows / 2);
        std::vector<vpPoint> point;
        point.resize(4);
        vpHomogeneousMatrix cMo;
        bool init = true;
        vpDetectorQRCode detector;
        vpImage<unsigned char> I;   // for gray images
        vpImageConvert::convert(img, I);
        bool qr_code_detected = detector.detect(I);
        if(qr_code_detected) {
                try {
                        YAML::Node node = YAML::Load(detector.getMessage(0));
                        if(node["size"]) {
                                qr_code_size = node["size"].as<float>();
                                point[0] = vpPoint(-qr_code_size/2.0f, -qr_code_size/2.0f, 0);
                                point[1] = vpPoint( qr_code_size/2.0f, -qr_code_size/2.0f, 0);
                                point[2] = vpPoint( qr_code_size/2.0f,  qr_code_size/2.0f, 0);
                                point[3] = vpPoint(-qr_code_size/2.0f,  qr_code_size/2.0f, 0);
                                qr_code_corners = detector.getPolygon(0);         // get the four corners location in the image
                                computePose(point, qr_code_corners, cam, init, cMo);         // resulting pose is available in cMo var
                                qr_code_pos = cMo.getTranslationVector().t();
                                qr_code_rot = vpQuaternionVector(cMo.getRotationMatrix());
                                // ROS_INFO_THROTTLE(5,"qr code with ID %d size %.1f detected, pos (%.1f %.1f %.1f)",
                                //                   ID,qr_code_size,qr_code_pos[0],qr_code_pos[1],qr_code_pos[2]);
                        }
                } catch(const YAML::ParserException& ex) {
                        ROS_ERROR_STREAM_THROTTLE(1,ex.what());
                        return false;
                }
                return true;
        }
        return false;
}

void Camera::changeResolution(int res){
        if(res==LOW_RES) {
                resolution = LOW_RES;
                cap.set(CAP_PROP_FRAME_WIDTH,lo_res.width);
                cap.set(CAP_PROP_FRAME_HEIGHT,lo_res.height);
        }else if(res==HIGH_RES) {
                resolution = HIGH_RES;
                cap.set(CAP_PROP_FRAME_WIDTH,hi_res.width);
                cap.set(CAP_PROP_FRAME_HEIGHT,hi_res.height);
        }
        ros::Duration d(0.1);
        d.sleep();
}
