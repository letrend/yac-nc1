#pragma once

#include <ros/ros.h>
#include <map>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace std;

class BrainDiceConfig {
public:
BrainDiceConfig();
/**
 * Reads a brain dice config file
 * @param filepath to config
 * @return success
 */
bool readConfig(const string &filepath){
        if(!fileExists(filepath)) {
                ROS_ERROR_STREAM(filepath << " does not exist, check your path");
                return false;
        }
        YAML::Node config = YAML::LoadFile(filepath);

        return true;
}
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
}
};
