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
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

sensor_msgs::JointState state;

void StatusReceiver(const sensor_msgs::JointStateConstPtr &msg){
  state = *msg;
}

int main()
{

  if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "visp_tracker");
  }
  ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
  ros::Subscriber motor_state = nh->subscribe("/motor/state",1,&StatusReceiver);
  ros::Publisher motor_command = nh->advertise<geometry_msgs::Vector3>("/motor/command",1);
  ros::AsyncSpinner spinner(0);
  spinner.start();

  bool try_again = true;
  while(try_again){
    try {
      cv::VideoCapture cap(0); // open the default camera
      cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
      cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
      if (!cap.isOpened()){ // check if we succeeded
        try_again = true;
        ROS_INFO_THROTTLE(1,"waiting for camera to become available");
        continue;
      }
      cv::Mat frame;
      int i = 0;
      while ((i++ < 100) && !cap.read(frame)) {
      }; // warm up camera by skiping unread frames
      ROS_INFO_STREAM("Image size: "
                << (int)cap.get(cv::CAP_PROP_FRAME_WIDTH) << " " << (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT));

      // Camera parameters should be adapted to your camera
      vpCameraParameters cam(640, 480, 640 / 2, 480 / 2);
      std::vector<vpPoint> point;
      float qr_code_side_length = 0.018; //0.00773
      point.push_back(vpPoint(-qr_code_side_length/2.0f, -qr_code_side_length/2.0f, 0)); // QRcode point 0 3D coordinates in plane Z=0
      point.push_back(vpPoint( qr_code_side_length/2.0f, -qr_code_side_length/2.0f, 0)); // QRcode point 1 3D coordinates in plane Z=0
      point.push_back(vpPoint( qr_code_side_length/2.0f,  qr_code_side_length/2.0f, 0)); // QRcode point 2 3D coordinates in plane Z=0
      point.push_back(vpPoint(-qr_code_side_length/2.0f,  qr_code_side_length/2.0f, 0)); // QRcode point 3 3D coordinates in plane Z=0
      vpHomogeneousMatrix cMo;
      bool init = true;
      vpDetectorQRCode detector;

      vpImage<unsigned char> I; // for gray images
      vpImageConvert::convert(frame, I);
      vpDisplayX d(I);
      while(1) {
        cap >> frame; // get a new frame from camera
        cv::Mat dst;               // dst must be a different Mat
        cv::flip(frame, dst, -1); // flip around both axis to align with cnc
        // Convert the image in ViSP format and display it
        vpImageConvert::convert(dst, I);
        vpDisplay::display(I);
        vpDisplay::flush(I);

        bool status = detector.detect(I);
        std::ostringstream legend;
        legend << detector.getNbObjects() << " bar code detected";
        vpDisplay::displayText(I, (int)I.getHeight() - 30, 10, legend.str(), vpColor::red);
        if (status) { // true if at least one QRcode is detected
          for (size_t i = 0; i < detector.getNbObjects(); i++) {
            std::vector<vpImagePoint> p = detector.getPolygon(i); // get the four corners location in the image

            for (size_t j = 0; j < p.size(); j++) {
              vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
              std::ostringstream number;
              number << j;
              vpDisplay::displayText(I, p[j] + vpImagePoint(15, 5), number.str(), vpColor::blue);
            }
            computePose(point, p, cam, init, cMo); // resulting pose is available in cMo var
            vpRowVector offset = cMo.getTranslationVector().t()*1000.0f;
            ROS_INFO_THROTTLE(1,"offset: %.3f %.3f",offset[0],offset[1]) ;
                      // << "Pose rotation (quaternion): " << vpQuaternionVector(cMo.getRotationMatrix()).t() << std::endl);
            vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
            if(p.size()==4){
              float qr_code_side_length_px = 0;
              qr_code_side_length_px += sqrtf(powf(p[1].get_u()-p[0].get_u(),2.0f)+powf(p[1].get_v()-p[0].get_v(),2.0f));
              qr_code_side_length_px += sqrtf(powf(p[2].get_u()-p[1].get_u(),2.0f)+powf(p[2].get_v()-p[1].get_v(),2.0f));
              qr_code_side_length_px += sqrtf(powf(p[3].get_u()-p[2].get_u(),2.0f)+powf(p[3].get_v()-p[2].get_v(),2.0f));
              qr_code_side_length_px += sqrtf(powf(p[0].get_u()-p[3].get_u(),2.0f)+powf(p[0].get_v()-p[3].get_v(),2.0f));
              ROS_INFO_THROTTLE(1,"%f pixel/mm",(qr_code_side_length_px/4.0f)/18.0f);
            }
            // if(!state.position.empty()){
            //   geometry_msgs::Vector3 msg;
            //   msg.x = state.position[0]-cMo[0][3]*1000.0f;
            //   msg.y = state.position[1]+cMo[1][3]*1000.0f;
            //   motor_command.publish(msg);
            // }
          }
        }
        vpDisplay::displayText(I, (int)I.getHeight() - 15, 10, "A click to quit...", vpColor::red);
        vpDisplay::flush(I);

        if (vpDisplay::getClick(I, false)){
          try_again = false;
          break;
        }
      }
    }
    catch(vpException e) {
      std::cout << "Catch an exception: " << e << std::endl;
    }
  }
  return 0;
}
