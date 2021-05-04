#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <pluginlib/class_list_macros.h>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QLabel>
#include <QMessageBox>
#include <cnc_gui/ui_cnc_gui.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Int32.h>
#include "brain_dice_config.hpp"
#include "camera.hpp"
#include "joystick.hpp"

#define PRE_DICE 0
#define POST_DICE 1
#define PRE_DISPENSE 2
#define POST_DISPENSE 3

#endif

using namespace std;

class CNCGUI : public rqt_gui_cpp::Plugin,
        public BrainDiceConfig,
        public Camera,
        public JoyStick {
Q_OBJECT
public:
CNCGUI();

virtual void initPlugin(qt_gui_cpp::PluginContext &context);

virtual void shutdownPlugin();

virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                          qt_gui_cpp::Settings &instance_settings) const;

virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                             const qt_gui_cpp::Settings &instance_settings);
public Q_SLOTS:
void calibrateCamera();
void calibrateCleanser();
void calibrate96well_0();
void calibrate96well_1();
void calibrateBrain();
void calibrateBackLash();
void scan();
void dry_run();
void run();
void pause();
void stop();
void prev_slice();
void next_slice();
void prev_cube();
void next_cube();
void auto_focus();
void lights();
void confirm();
void clean();
void reloadConfig();

void move();
void zero();

void plotData();
void drawScanArea();
void drawHUD(int resolution);
void dicingConfigUpdate();
void buttonStateUpdate();
void planUpdate();
void planUpdate(int write);
void autoPlan();

Q_SIGNALS:
void scanAreaUpdate();
void HUDupdate(int resolution);
void updateMotorState();
void updateButtonStates();
void updateDicingConfig();
void updatePlan(int write);

private:
void CalibrateCameraThread();
void CalibrateCleanserThread();
void Calibrate96wellThread(int number=0);
void CalibrateBrainThread();
void DryRunThread();
void RunThread();
void BackLashCalibrationThread();
void StatusReceiver(const sensor_msgs::JointStateConstPtr &msg);
void FrameGrabberThread();
void ScanThread();
void Clean();
void JoyStickContolThread();
void CubeShot(int type);
void MoveTool(float z);
bool MoveToolSave(float z, float error = 0.1, int timeout_sec=3);
void loadPlanImage();
void calculate96wellPositions();
float trackQRCode(float x=0, float y=0);
void AutoFocusMonitorThread();
bool WaitForPositionReachedSave(geometry_msgs::Vector3 &msg, float error = 0.1, float timeout=1);
void WaitForPositionReached(geometry_msgs::Vector3 &msg, float error = 0.1, float timeout=1);
void Stitch(int resolution, float x, float y, Mat &img);
void ScanStitch(int resolution = LOW_RES);
void ScanStitch(Mat &img, int resolution = LOW_RES);
bool stitch(int slice);
void drawHUD(Mat &img_src, Mat &img_dst, int resolution=LOW_RES);
void drawPlan();
void drawLine(cv::Mat &img, int x0, int y0, int x1, int y1, cv::Scalar color, int thickness=1);
string exec(const char* cmd);
int getV4L2(string value);
void setV4L2(string name, int value);
bool eventFilter( QObject* watched, QEvent* event );
void drawImage(Mat &img, QLabel* label);
bool checkConfirm(int timeout_sec=30);
bool get96well(int cube, Scalar &color);
bool checkPause();
bool checkStop();
string getDateString();
private:
Ui::CNCGUI ui;
QColor color_pallette[16] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                             Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray, Qt::green, Qt::cyan};
QWidget *widget_;
ros::NodeHandlePtr nh;
ros::ServiceClient zero_srv;
ros::Publisher motor_command, neopixel_all_pub, cleanser_pub;
ros::Subscriber status_subscriber;
map<string,QVector<double> > values;
boost::shared_ptr<ros::AsyncSpinner> spinner;
QString motorConfigFile;
map<string,uint16_t> messages_received;
boost::shared_ptr<std::thread> frame_grabber_thread, scan_thread,
                               qr_code_tracking_thread, qr_code_detection_thread,
                               backlash_calibration_thread, auto_focus_monitoring_thread,
                               calibrate_camera_thread, calibrate_cleanser_thread,
                               calibrate_96well_thread, calibrate_brain_thread,
                               plan_dicing_thread, joystick_control_thread,
                               dry_run_thread, run_thread;
ros::Time start_time;
vector<Mat> frame, frame_reticle, cnc_area_image, cnc_area_image_reticle,
            scan_area_image, plan_image, plan_image_reticle;
bool qr_code_detected = false, grab_frames = true;
float qr_tracking_error = 10;
vpRowVector qr_code_pos, qr_code_target;
vpQuaternionVector qr_code_rot;
vector<vpImagePoint> qr_code_corners;
float qr_code_size = 20;
mutex mux;
QString brainDiceConfigFile;
Mat cameraMatrix, distCoeffs;
QPoint mouse_cursor_cnc_area, mouse_cursor_camera, mouse_cursor_plan_area;
vector<Point2f> cubes;
Size cubes_dim;
vector<bool> cube_active;
int cube_hovered = 0, cube_target = -1;
int plan_operation = 0;
vector<unsigned long> ninety_six_well_IDs;
vector<vector<int> > ninety_six_well_content;
int well_counter = 0;
vector<Point2f> ninety_six_well_pos;
bool tool_toggle = false;
bool pause_active = false, stop_active = false;
};
