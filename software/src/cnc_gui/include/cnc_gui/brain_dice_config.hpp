#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <map>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <stdio.h>
#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;
namespace fs = boost::filesystem;
using namespace cv;

#define LOW_RES 0
#define HIGH_RES 1

class BrainDiceConfig {
public:
BrainDiceConfig(){
        if (!ros::isInitialized()) {
                ros::Time::init();
        }
};
/**
 * Reads a brain dice config file
 * @param filepath to config
 * @return success
 */
bool readConfig(const string &filepath);
/**
 * Writes a brain dice config file
 * @param filepath
 * @return success
 */
bool writeConfig(const string &filepath);
/**
 * Checks if a file exists
 * @param filepath
 * @return exists
 */
inline bool fileExists(const string &filepath){
        struct stat buffer;
        return (stat (filepath.c_str(), &buffer) == 0);
};

bool findConfigFile(fs::path dir, fs::path &config);

bool checkSliceConfig(fs::path path);
bool writeSliceConfig(fs::path path, YAML::Node config);

YAML::Node generateDefaultSliceConfig();

bool clearSlice(int slice);
bool sliceExists(int slice);

bool saveImage(int resolution, int slice, float x, float y,
               Mat &img_raw, Mat &img_undistorted);

bool getSlice(int resolution, int slice, vector<float> &x, vector<float> &y,
              vector<unsigned long> &timestamps, vector<Mat> &images, bool type=false);

bool readPlan(int slice, unsigned long &timestamp, Size &dims, vector<Point2f> &positions,
              vector<bool> &position_active, vector<unsigned long> &ninety_six_well_IDs,
              vector<vector<int> > &ninety_six_well_content );
bool writePlan(int slice, unsigned long timestamp, Size &dims, vector<Point2f> &positions,
               vector<bool> &position_active, vector<unsigned long> &ninety_six_well_IDs,
               vector<vector<int> > &ninety_six_well_content );

YAML::Node config;
fs::path config_file_path, root_folder;
map<int,fs::path> slice_config_filepath;
map<int,YAML::Node> slice_config;
geometry_msgs::Vector3 optical_reference_pos,
                       brain_sample_top_left, brain_sample_bottom_right,
                       cleanser_pos;
vector<geometry_msgs::Vector3> ninety_six_well_top_left, ninety_six_well_bottom_right;
float ninety_six_well_distance = 0;
int slice = 0, cube = 0, camera_id = 0;
int ninety_six_well_counter = 0;
string camera_calibration_file_path_lo_res = "",
       camera_calibration_file_path_hi_res = "";
vector<float> pixel_per_mm_x = {20,20}, pixel_per_mm_y = {20,20};
int focus_absolute = 60, exposure_absolute = 100;
float cnc_x_dim = 0, cnc_y_dim = 0;
float tool_camera_offset_x = 0, tool_camera_offset_y = 0;
int tool_type = 0;
float tool_size = 5;
float cut_depth = 0, dwell_on_surface = 0, dispense_depth = 0;
int dispense_repetitions = 0;
};
