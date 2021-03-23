#include "brain_dice_config.hpp"

class InputParser {
public:
InputParser (int &argc, char **argv){
        for (int i=1; i < argc; ++i)
                this->tokens.push_back(std::string(argv[i]));
}
/// @author iain
const std::string& getCmdOption(const std::string &option) const {
        std::vector<std::string>::const_iterator itr;
        itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
        if (itr != this->tokens.end() && ++itr != this->tokens.end()) {
                return *itr;
        }
        static const std::string empty_string("");
        return empty_string;
}
/// @author iain
bool cmdOptionExists(const std::string &option) const {
        return std::find(this->tokens.begin(), this->tokens.end(), option)
               != this->tokens.end();
}
private:
std::vector <std::string> tokens;
};

int main(int argc, char *argv[]){
        InputParser input(argc, argv);

        const std::string &filepath = input.getCmdOption("-c");
        if (filepath.empty()) {
                ROS_INFO("please provide brain dice config filepath with -c");
                return -1;
        }

        Mat img = imread("src/cnc_gui/calibration/brain.png", IMREAD_COLOR);

        BrainDiceConfig c;
        c.readConfig(filepath);
        c.clearSlice(1);
        vector<float> x = {0.1,0.2,0.3}, y = {0.3,0.4,0.5};
        for(int slice=0; slice<=2; slice++) {
                for(int frame_index=0; frame_index<2; frame_index++) {
                        c.saveImage(LOW_RES,slice,x[frame_index],y[frame_index],img,img);
                        c.saveImage(HIGH_RES,slice,x[frame_index],y[frame_index],img,img);
                }
        }
        for(int slice=0; slice<=2; slice++) {
                vector<float> x,y;
                vector<unsigned long> timestamps;
                vector<Mat> images;
                if(c.getSlice(LOW_RES,slice,x,y,timestamps,images)) {
                        int i=0;
                        for(auto val:timestamps) {
                                ROS_INFO_STREAM(val << " size: " << images[i].size);
                                i++;
                        }
                }

                if(c.getSlice(HIGH_RES,slice,x,y,timestamps,images)) {
                        int i=0;
                        for(auto val:timestamps) {
                                ROS_INFO_STREAM(val << " size: " << images[i].size);
                                i++;
                        }
                }
                if(c.getSlice(LOW_RES,slice,x,y,timestamps,images,true)) {
                        int i=0;
                        for(auto val:timestamps) {
                                ROS_INFO_STREAM(val << " size: " << images[i].size);
                                i++;
                        }
                }

                if(c.getSlice(HIGH_RES,slice,x,y,timestamps,images,true)) {
                        int i=0;
                        for(auto val:timestamps) {
                                ROS_INFO_STREAM(val << " size: " << images[i].size);
                                i++;
                        }
                }
        }
}
