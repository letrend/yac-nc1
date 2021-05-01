#include "brain_dice_config.hpp"

bool BrainDiceConfig::readConfig(const string &filepath){
        if(!fileExists(filepath)) {
                ROS_ERROR_STREAM(filepath << " does not exist, check your path");
                return false;
        }
        try {
                ROS_INFO_STREAM("reading config " << filepath);
                config = YAML::LoadFile(filepath);
                vector<float> x = config["ninety_six_well"]["top_left"]["x"].as<vector<float> >();
                vector<float> y = config["ninety_six_well"]["top_left"]["y"].as<vector<float> >();
                vector<float> z = config["ninety_six_well"]["top_left"]["z"].as<vector<float> >();
                for(int i=0; i<x.size(); i++) {
                        geometry_msgs::Vector3 msg;
                        msg.x = x[i];
                        msg.y = y[i];
                        msg.z = z[i];
                        ninety_six_well_top_left.push_back(msg);
                }
                x = config["ninety_six_well"]["bottom_right"]["x"].as<vector<float> >();
                y = config["ninety_six_well"]["bottom_right"]["y"].as<vector<float> >();
                z = config["ninety_six_well"]["bottom_right"]["z"].as<vector<float> >();
                for(int i=0; i<x.size(); i++) {
                        geometry_msgs::Vector3 msg;
                        msg.x = x[i];
                        msg.y = y[i];
                        msg.z = z[i];
                        ninety_six_well_bottom_right.push_back(msg);
                }
                ninety_six_well_distance = config["ninety_six_well"]["distance"].as<float>();
                brain_sample_top_left.x = config["brain_sample"]["top_left"]["x"].as<float>();
                brain_sample_top_left.y = config["brain_sample"]["top_left"]["y"].as<float>();
                brain_sample_top_left.z = config["brain_sample"]["top_left"]["z"].as<float>();
                brain_sample_bottom_right.x = config["brain_sample"]["bottom_right"]["x"].as<float>();
                brain_sample_bottom_right.y = config["brain_sample"]["bottom_right"]["y"].as<float>();
                brain_sample_bottom_right.z = config["brain_sample"]["bottom_right"]["z"].as<float>();
                slice = config["dicing_state"]["slice"].as<int>();
                cube = config["dicing_state"]["cube"].as<int>();
                ninety_six_well_counter = config["dicing_state"]["well_counter"].as<int>();
                camera_id = config["camera"]["id"].as<int>();
                camera_calibration_file_path_lo_res =
                        config["camera"]["calibration_file_path"]["lo_res"].as<string>();
                camera_calibration_file_path_hi_res =
                        config["camera"]["calibration_file_path"]["hi_res"].as<string>();
                pixel_per_mm_x[0] = config["camera"]["pixel_per_mm"]["lo_res"]["x"].as<float>();
                pixel_per_mm_y[0] = config["camera"]["pixel_per_mm"]["lo_res"]["y"].as<float>();
                pixel_per_mm_x[1] = config["camera"]["pixel_per_mm"]["hi_res"]["x"].as<float>();
                pixel_per_mm_y[1] = config["camera"]["pixel_per_mm"]["hi_res"]["y"].as<float>();
                focus_absolute = config["camera"]["focus_absolute"].as<int>();
                exposure_absolute = config["camera"]["exposure_absolute"].as<int>();
                cnc_x_dim = config["cnc"]["dimension"]["x"].as<float>();
                cnc_y_dim = config["cnc"]["dimension"]["y"].as<float>();
                tool_camera_offset_x = config["tool"]["camera_offset"]["x"].as<float>();
                tool_camera_offset_y = config["tool"]["camera_offset"]["y"].as<float>();
                cut_depth = config["tool"]["cut_depth"].as<float>();
                dwell_on_surface = config["tool"]["dwell_on_surface"].as<float>();
                dispense_depth = config["tool"]["dispense_depth"].as<float>();
                dispense_repetitions = config["tool"]["dispense_repetitions"].as<int>();
                tool_size = config["tool"]["size"].as<float>();
                tool_type = config["tool"]["type"].as<int>();
                optical_reference_pos.x = config["optical_reference"]["pos"]["x"].as<float>();
                optical_reference_pos.y = config["optical_reference"]["pos"]["y"].as<float>();
                optical_reference_pos.z = config["optical_reference"]["pos"]["z"].as<float>();
                cleanser_pos.x = config["cleanser"]["pos"]["x"].as<float>();
                cleanser_pos.y = config["cleanser"]["pos"]["y"].as<float>();
                cleanser_pos.z = config["cleanser"]["pos"]["z"].as<float>();
                cleanser_time = config["cleanser"]["time"].as<float>();
                cleanser_intensity = config["cleanser"]["intensity"].as<float>();
                cleanser_strategy = config["cleanser"]["strategy"].as<int>();
                cleanser_frequency = config["cleanser"]["frequency"].as<int>();
                cleanser_tool_depth = config["cleanser"]["tool_depth"].as<float>();
                cleanser_dwell_after_clean = config["cleanser"]["dwell_after_clean"].as<float>();
                config_file_path = fs::path(filepath);
                root_folder = config_file_path.parent_path();
                ROS_INFO_STREAM("root folder: " <<root_folder.string());
                for(auto & child : fs::directory_iterator( root_folder )) {
                        if(!fs::is_regular_file(child)) { // only check folders
                                fs::path config;
                                if(findConfigFile(child,config)) {
                                        if(checkSliceConfig(config))
                                                ROS_INFO_STREAM("folder " << child << " contains config: " << config);
                                }
                        }
                }
        } catch(const YAML::ParserException& ex) {
                ROS_FATAL_STREAM(ex.what()<<endl<<"oh oh, something went wrong reading you config file!!! terminating...");
                return false;
        }

        return true;
}

bool BrainDiceConfig::writeConfig(const string &filepath){
        std::ofstream fout(filepath);
        if (!fout.is_open()) {
                ROS_WARN_STREAM("Could not write config " << filepath);
                return false;
        }
        for(int i=0; i<ninety_six_well_top_left.size(); i++) {
                config["ninety_six_well"]["top_left"]["x"][i] = ninety_six_well_top_left[i].x;
                config["ninety_six_well"]["top_left"]["y"][i] = ninety_six_well_top_left[i].y;
                config["ninety_six_well"]["top_left"]["z"][i] = ninety_six_well_top_left[i].z;
                config["ninety_six_well"]["bottom_right"]["x"][i] = ninety_six_well_bottom_right[i].x;
                config["ninety_six_well"]["bottom_right"]["y"][i] = ninety_six_well_bottom_right[i].y;
                config["ninety_six_well"]["bottom_right"]["z"][i] = ninety_six_well_bottom_right[i].z;
        }
        config["brain_sample"]["top_left"]["x"] = brain_sample_top_left.x;
        config["brain_sample"]["top_left"]["y"] = brain_sample_top_left.y;
        config["brain_sample"]["top_left"]["z"] = brain_sample_top_left.z;
        config["brain_sample"]["bottom_right"]["x"] = brain_sample_bottom_right.x;
        config["brain_sample"]["bottom_right"]["y"] = brain_sample_bottom_right.y;
        config["brain_sample"]["bottom_right"]["z"] = brain_sample_bottom_right.z;
        config["dicing_state"]["slice"] = slice;
        config["dicing_state"]["cube"] = cube;
        config["dicing_state"]["well_counter"] = ninety_six_well_counter;
        config["camera"]["pixel_per_mm"]["lo_res"]["x"] = pixel_per_mm_x[0];
        config["camera"]["pixel_per_mm"]["lo_res"]["y"] = pixel_per_mm_y[0];
        config["camera"]["pixel_per_mm"]["hi_res"]["x"] = pixel_per_mm_x[1];
        config["camera"]["pixel_per_mm"]["hi_res"]["y"] = pixel_per_mm_y[1];
        config["camera"]["focus_absolute"] = focus_absolute;
        config["camera"]["exposure_absolute"] = exposure_absolute;
        config["optical_reference"]["pos"]["x"] = optical_reference_pos.x;
        config["optical_reference"]["pos"]["y"] = optical_reference_pos.y;
        config["optical_reference"]["pos"]["z"] = optical_reference_pos.z;
        config["cleanser"]["pos"]["x"] = cleanser_pos.x;
        config["cleanser"]["pos"]["y"] = cleanser_pos.y;
        config["cleanser"]["pos"]["z"] = cleanser_pos.z;
        config["cleanser"]["tool_depth"] = cleanser_tool_depth;
        fout << config;

        return true;
}

bool BrainDiceConfig::findConfigFile(fs::path dir, fs::path &config){
        for(auto & f : fs::directory_iterator( dir )) {
                if(fs::is_regular_file(f) && fs::extension(f)==".yaml") {
                        config = f;
                        return true;
                }
        }
        return false;
}

bool BrainDiceConfig::checkSliceConfig(fs::path path){
        try {
                YAML::Node c = YAML::LoadFile(path.string());
                int slice = stoi(path.stem().string());
                fs::path root_dir = path.parent_path();
                for(int resolution = 0; resolution <=1; resolution++) {
                        vector<unsigned long> timestamps;
                        vector<float> x, y;
                        if(resolution==LOW_RES) {
                                timestamps = c["scan"]["lo_res"]["time"].as<vector<unsigned long> >();
                                x = c["scan"]["lo_res"]["cnc_coordinates"]["x"].as<vector<float> >();
                                y = c["scan"]["lo_res"]["cnc_coordinates"]["y"].as<vector<float> >();
                        }else if(resolution==HIGH_RES) {
                                timestamps = c["scan"]["hi_res"]["time"].as<vector<unsigned long> >();
                                x = c["scan"]["hi_res"]["cnc_coordinates"]["x"].as<vector<float> >();
                                y = c["scan"]["hi_res"]["cnc_coordinates"]["y"].as<vector<float> >();
                        }
                        int number_of_timestamps = timestamps.size();
                        if(number_of_timestamps!=x.size() || number_of_timestamps!=y.size()) {
                                ROS_ERROR("the number of timestamps %ld does not match the number of cnc_coordinates (%ld/%ld)",
                                          timestamps.size(),x.size(),y.size());
                                return false;
                        }
                        for(auto v:timestamps) {
                                if(resolution==LOW_RES) {
                                        fs::path lo_res_path_raw = root_dir/("lo_res/raw/"+to_string(v)+".png");
                                        if(!fs::exists(lo_res_path_raw)) {
                                                ROS_ERROR_STREAM("lo_res raw image does not exist: " << lo_res_path_raw);
                                                return false;
                                        }
                                        fs::path lo_res_path_undistorted = root_dir/("lo_res/undistorted/"+to_string(v)+".png");
                                        if(!fs::exists(lo_res_path_undistorted)) {
                                                ROS_ERROR_STREAM("lo_res undistorted image does not exist: " << lo_res_path_undistorted);
                                                return false;
                                        }
                                }else if(resolution==HIGH_RES) {
                                        fs::path hi_res_path_raw = root_dir/("hi_res/raw/"+to_string(v)+".png");
                                        if(!fs::exists(hi_res_path_raw)) {
                                                ROS_ERROR_STREAM("hi_res raw image does not exist: " << hi_res_path_raw);
                                                return false;
                                        }
                                        fs::path hi_res_path_undistorted = root_dir/("hi_res/undistorted/"+to_string(v)+".png");
                                        if(!fs::exists(hi_res_path_undistorted)) {
                                                ROS_ERROR_STREAM("hi_res undistorted image does not exist: " << hi_res_path_undistorted);
                                                return false;
                                        }
                                }
                        }
                }
                slice_config_filepath[slice] = path;
                slice_config[slice] = c;
        } catch(const YAML::ParserException& ex) {
                ROS_FATAL_STREAM(ex.what()<<endl<<path << endl <<
                                 "oh oh, something went wrong reading your config file!!! terminating...");
                return false;
        }
        return true;
}

bool BrainDiceConfig::writeSliceConfig(fs::path path, YAML::Node config){
        ofstream fout(path.string());
        if (!fout.is_open()) {
                ROS_WARN_STREAM("Could not write config " << path);
                return false;
        }
        fout << config;
        return true;
}

YAML::Node BrainDiceConfig::generateDefaultSliceConfig(){
        string s = {
                "scan:\n"
                "  lo_res:\n"
                "    time: []\n"
                "    cnc_coordinates:\n"
                "      x: []\n"
                "      y: []\n"
                "  hi_res:\n"
                "    time: []\n"
                "    cnc_coordinates:\n"
                "      x: []\n"
                "      y: []\n"
                "plan:\n"
                "  time: 0\n"
                "  width: 0\n"
                "  height: 0\n"
                "  number_of_positions: 0\n"
                "  number_of_active_positions: 0\n"
                "  number_of_96wells: 0\n"
                "  ninety_six_well_IDs: []\n"
                "  ninety_six_well_content: []\n"
                "  cnc_coordinates:\n"
                "    x: []\n"
                "    y: []\n"
        };
        YAML::Node c = YAML::Load(s);
        return c;
}

bool BrainDiceConfig::clearSlice(int slice){
        if (!sliceExists(slice)) { // slice does not exist yet
                return true;
        }
        ROS_INFO_STREAM("deleting " << slice_config_filepath[slice].parent_path());
        fs::remove_all(slice_config_filepath[slice].parent_path());
        slice_config_filepath.erase(slice);
        slice_config.erase(slice);
        return true;
}

bool BrainDiceConfig::sliceExists(int slice){
        if (slice_config.count(slice)) {
                return true;
        }
        return false;
}

bool BrainDiceConfig::saveImage(int resolution, int slice, float x, float y,
                                Mat &img_raw, Mat &img_undistorted){
        unsigned long t0 = ros::Time::now().toNSec();
        if (!slice_config.count(slice)) {
                ROS_INFO("slice config of %d does not exist yet, creating it...", slice);
                YAML::Node c = generateDefaultSliceConfig();
                // ROS_INFO_STREAM(c);
                ROS_INFO("creating folder structure");
                fs::path slice_folder_path = root_folder/std::to_string(slice);
                if(fs::exists(slice_folder_path)) {
                        ROS_INFO_STREAM("\tslice folder exists already: " << slice_folder_path);
                }else{
                        ROS_INFO_STREAM("\tslice folder does not exist yet, creating it: " << slice_folder_path);
                        fs::create_directory(slice_folder_path);
                        ROS_INFO("\t\tcreating subdirectories");
                        fs::create_directory(slice_folder_path/string("lo_res"));
                        fs::create_directory(slice_folder_path/string("lo_res")/string("raw"));
                        fs::create_directory(slice_folder_path/string("lo_res")/string("undistorted"));
                        fs::create_directory(slice_folder_path/string("hi_res"));
                        fs::create_directory(slice_folder_path/string("hi_res")/string("raw"));
                        fs::create_directory(slice_folder_path/string("hi_res")/string("undistorted"));
                }
                slice_config[slice] = c;
                fs::path filepath = slice_folder_path/(to_string(slice)+".yaml");
                slice_config_filepath[slice] = filepath;
        }

        fs::path output_file;
        if(resolution==LOW_RES) {
                output_file = root_folder/to_string(slice)/string("lo_res");
        }else if(resolution==HIGH_RES) {
                output_file = root_folder/to_string(slice)/string("hi_res");
        }
        fs::path output_file_raw, output_file_undistorted;
        output_file_raw = output_file/string("raw");
        output_file_undistorted = output_file/string("undistorted");
        if(fs::exists(output_file_raw) && fs::exists(output_file_undistorted)) {
                output_file_raw/=(to_string(t0)+".png");
                imwrite(output_file_raw.string(), img_raw);
                output_file_undistorted/=(to_string(t0)+".png");
                imwrite(output_file_undistorted.string(), img_undistorted);
                if(resolution==LOW_RES) {
                        slice_config[slice]["scan"]["lo_res"]["time"].push_back(t0);
                        slice_config[slice]["scan"]["lo_res"]["cnc_coordinates"]["x"].push_back(x);
                        slice_config[slice]["scan"]["lo_res"]["cnc_coordinates"]["y"].push_back(y);
                }else if(resolution==HIGH_RES) {
                        slice_config[slice]["scan"]["hi_res"]["time"].push_back(t0);
                        slice_config[slice]["scan"]["hi_res"]["cnc_coordinates"]["x"].push_back(x);
                        slice_config[slice]["scan"]["hi_res"]["cnc_coordinates"]["y"].push_back(y);
                }
                return writeSliceConfig(slice_config_filepath[slice],slice_config[slice]);
        }
        return false;
}

bool BrainDiceConfig::getSlice(int resolution, int slice, vector<float> &x, vector<float> &y,
                               vector<unsigned long> &timestamps, vector<Mat> &images, bool type){
        if (!slice_config.count(slice)) {                          // slice does not exist yet
                return false;
        }

        if(resolution==LOW_RES) {
                timestamps = slice_config[slice]["scan"]["lo_res"]["time"].as<vector<unsigned long> >();
                x = slice_config[slice]["scan"]["lo_res"]["cnc_coordinates"]["x"].as<vector<float> >();
                y = slice_config[slice]["scan"]["lo_res"]["cnc_coordinates"]["y"].as<vector<float> >();
        }else if(resolution==HIGH_RES) {
                timestamps = slice_config[slice]["scan"]["hi_res"]["time"].as<vector<unsigned long> >();
                x = slice_config[slice]["scan"]["hi_res"]["cnc_coordinates"]["x"].as<vector<float> >();
                y = slice_config[slice]["scan"]["hi_res"]["cnc_coordinates"]["y"].as<vector<float> >();
        }
        fs::path root_dir = slice_config_filepath[slice].parent_path();
        for(auto val:timestamps) {
                fs::path image_file_path;
                image_file_path  = root_dir/(string((resolution==LOW_RES ? "lo_res" : "hi_res"))+
                                             string((type ? "/raw/" : "/undistorted/"))+to_string(val)+".png");
                if(fs::exists(image_file_path)) {
                        ROS_INFO_STREAM("loading: " << image_file_path);
                        images.push_back(imread(image_file_path.string(),IMREAD_COLOR));
                }else{
                        ROS_ERROR_STREAM("file does not exist: " << image_file_path);
                        return false;
                }
        }
        return true;
}

bool BrainDiceConfig::readPlan(int slice, unsigned long &timestamp,Size &dims,
                               vector<Point2f> &positions, vector<bool> &position_active,
                               vector<unsigned long> &ninety_six_well_IDs,
                               vector<vector<int> > &ninety_six_well_content ){
        try {
                if (!slice_config.count(slice)) {                    // slice does not exist yet
                        return false;
                }
                timestamp = slice_config[slice]["plan"]["time"].as<unsigned long>();
                if(timestamp==0)
                        return false;
                dims.width = slice_config[slice]["plan"]["width"].as<int>();
                dims.height = slice_config[slice]["plan"]["height"].as<int>();
                int number_of_positions = slice_config[slice]["plan"]["number_of_positions"].as<int>();
                vector<float> x = slice_config[slice]["plan"]["cnc_coordinates"]["x"].as<vector<float> >();
                vector<float> y = slice_config[slice]["plan"]["cnc_coordinates"]["y"].as<vector<float> >();
                position_active = slice_config[slice]["plan"]["position_active"].as<vector<bool> >();
                positions.clear();
                for(int i=0; i<x.size(); i++) {
                        positions.push_back(Point2f(x[i],y[i]));
                }
                ninety_six_well_IDs = slice_config[slice]["plan"]["ninety_six_well_IDs"].as<vector<unsigned long> >();
                ninety_six_well_content = slice_config[slice]["plan"]["ninety_six_well_content"].as<vector<vector<int> > >();
                if(positions.size()!=number_of_positions || position_active.size()!=number_of_positions) {
                        ROS_ERROR("number_of_positions %d does not match number of positions %ld/%ld in plan",
                                  number_of_positions, positions.size(), position_active.size());
                        positions.clear();
                        position_active.clear();
                        return false;
                }
                return true;
        } catch(const YAML::ParserException& ex) {
                ROS_ERROR_STREAM(ex.what() <<
                                 "oh oh, something went wrong reading your plan!!! terminating...");
                return false;
        }
}

bool BrainDiceConfig::writePlan(int slice, unsigned long timestamp, Size &dims,
                                vector<Point2f> &positions, vector<bool> &position_active,
                                vector<unsigned long> &ninety_six_well_IDs,
                                vector<vector<int> > &ninety_six_well_content ){
        try {
                if (!slice_config.count(slice)) {                    // slice does not exist yet
                        return false;
                }
                slice_config[slice]["plan"]["time"] = timestamp;
                slice_config[slice]["plan"]["width"] = dims.width;
                slice_config[slice]["plan"]["height"] = dims.height;
                slice_config[slice]["plan"]["number_of_positions"] = positions.size();
                int number_of_active_positions = 0;
                for(auto val:position_active) {
                        if(val)
                                number_of_active_positions++;
                }
                slice_config[slice]["plan"]["number_of_active_positions"] = number_of_active_positions;
                slice_config[slice]["plan"]["cnc_coordinates"]["x"] = vector<float>();
                slice_config[slice]["plan"]["cnc_coordinates"]["y"] = vector<float>();
                slice_config[slice]["plan"]["position_active"] = position_active;
                slice_config[slice]["plan"]["ninety_six_well_IDs"] = ninety_six_well_IDs;
                slice_config[slice]["plan"]["ninety_six_well_content"] = ninety_six_well_content;

                for(int i=0; i<position_active.size(); i++) {
                        slice_config[slice]["plan"]["cnc_coordinates"]["x"].push_back(positions[i].x);
                        slice_config[slice]["plan"]["cnc_coordinates"]["y"].push_back(positions[i].y);
                }

                return writeSliceConfig(slice_config_filepath[slice],slice_config[slice]);
        } catch(const YAML::ParserException& ex) {
                ROS_ERROR_STREAM(ex.what() <<
                                 "oh oh, something went wrong writing your plan!!! terminating...");
                return false;
        }
}
