#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <regex>

using namespace cv;
using namespace std;

string exec(const char* cmd) {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe) {
                throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
                result += buffer.data();
        }
        return result;
}

int getV4L2(string value){
        string val = exec(("v4l2-ctl -C " + value).c_str());
        string output = std::regex_replace(
                val,
                std::regex("[^0-9]*([0-9]+).*"),
                std::string("$1")
                );
        return std::stoi (output,nullptr,0);
}

void setV4L2(string name, int value){
        exec(("v4l2-ctl -c " + name + "=" + std::to_string(value)).c_str());
}

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

int main(int argc, char* argv[])
{
        InputParser input(argc, argv);



        Mat frame, frame_undistorted, frame_small, frame_undistorted_small;
        //--- INITIALIZE VIDEOCAPTURE
        VideoCapture cap;
        Mat cameraMatrix, distCoeffs;
        const std::string &filename = input.getCmdOption("-c");
        bool calib = false;
        if (!filename.empty()) {
                FileStorage fs;
                fs.open(filename, FileStorage::READ);
                if (!fs.isOpened())
                {
                        cerr << "Failed to open " << filename << endl;
                        return 1;
                }

                fs["camera_matrix"] >> cameraMatrix;
                fs["distortion_coefficients"] >> distCoeffs;
                cout << cameraMatrix << endl;
                cout << distCoeffs << endl;
                calib = true;
        }

        const std::string &output_path = input.getCmdOption("-o");
        if (!output_path.empty()) {
                cout << "using output path " << output_path << endl;
        }

        // open the default camera using default API
        // cap.open(0);
        // OR advance usage: select any API backend
        int deviceID = 0;         // 0 = open default camera
        int apiID = cv::CAP_ANY;  // 0 = autodetect default API
        // open selected camera using selected API
        cap.open(deviceID, apiID);
        const std::string &width = input.getCmdOption("-w");
        const std::string &height = input.getCmdOption("-h");
        if (width.empty() || height.empty()) {
                cout << "using default image size" << endl;
        }else{
                cout << "using image size " << width << " " << height << endl;
                cap.set(CAP_PROP_FRAME_WIDTH,stoi(width));
                cap.set(CAP_PROP_FRAME_HEIGHT,stoi(height));
        }

        // check if we succeeded
        if (!cap.isOpened()) {
                cerr << "ERROR! Unable to open camera\n";
                return -1;
        }
        setV4L2("focus_auto",1);
        setV4L2("exposure_auto",3);
        //--- GRAB AND WRITE LOOP
        cout << "Start grabbing" << endl
             << "Press esc to terminate" << endl
             << "Press space to take a picture" << endl;
        int frame_index = 0;
        for (;;)
        {
                // wait for a new frame from camera and store it into 'frame'
                cap.read(frame);
                // check if we succeeded
                if (frame.empty()) {
                        cerr << "ERROR! blank frame grabbed\n";
                        break;
                }

                // show live and wait for a key with timeout long enough to show images



                resize(frame, frame_small, cv::Size(640,480), 0, 0, INTER_LINEAR);
                imshow("Live", frame_small);
                if(calib) {
                        undistort(frame, frame_undistorted, cameraMatrix, distCoeffs);
                        resize(frame_undistorted, frame_undistorted_small, cv::Size(640,480), 0, 0, INTER_LINEAR);
                        imshow("undistorted", frame_undistorted_small);
                }

                switch (waitKey(1)) {
                case 27:
                        // the camera will be deinitialized automatically in VideoCapture destructor
                        return 0;
                        break;
                case 32:
                        if(output_path.empty()) {
                                cout << "you didnt tell me where to save, use -o" << endl;
                        }else{
                                char str[20];
                                sprintf(str, "img%.3d.png",frame_index);
                                imwrite(output_path+"/"+str, frame);
                                cout << output_path+"/"+str << " saved" << endl;
                                frame_index++;
                        }
                        break;
                }

        }


}
