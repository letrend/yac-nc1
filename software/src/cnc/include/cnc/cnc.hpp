#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <control_msgs/JointControllerState.h>
#include <control_msgs/PidState.h>
#include <thread>
#include <stdlib.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Int32.h>

using namespace std;

#define IORD(base,reg) (*(((volatile uint32_t*)base)+reg))
#define IOWR(base,reg,data) (*(((volatile uint32_t*)base)+reg)=data)

#define POSITION 0
#define MM_PER_TICK 0.000486f

class CNC {
public:
CNC(vector<int32_t*> stepper_base, int32_t *end_switches, int32_t *neopixel) :
        stepper_base(stepper_base),end_switches(end_switches),neopixel(neopixel){
        if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "cnc", ros::init_options::NoSigintHandler);
                ros::start();
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        motor_state_pub = nh->advertise<sensor_msgs::JointState>("/motor/state",1);
        pid_state_pub = nh->advertise<control_msgs::PidState>("/motor/controller_state",1);
        endswitch_pub = nh->advertise<geometry_msgs::Vector3>("/motor/endswitch",1);
        position_sub = nh->subscribe("/motor/command",1,&CNC::PositionCommandCallback,this);
        neopixel_single_sub = nh->subscribe("/neopixel/single",1,&CNC::NeopixelSingleCallback,this);
        neopixel_all_sub = nh->subscribe("/neopixel/all",1,&CNC::NeopixelAllCallback,this);
        cleanser_sub = nh->subscribe("/cleanser",1,&CNC::CleanserCallback,this);

        number_of_motors = stepper_base.size();

        position_offset.resize(number_of_motors);

        // motor axis initialization
        for(int i=number_of_axis-1; i>=0; i--) {
                IOWR(stepper_base[i],REGISTER::ms,STEP_16);
                IOWR(stepper_base[i],REGISTER::ramp_up_limit,ramp_up_limits[i]);
                IOWR(stepper_base[i],REGISTER::ramp_up_threshold,ramp_up_thresholds[i]);
                IOWR(stepper_base[i],REGISTER::Kp,1);
                IOWR(stepper_base[i],REGISTER::Ki,1);
                IOWR(stepper_base[i],REGISTER::deadband,20);
                IOWR(stepper_base[i],REGISTER::integralMax,10);
                IOWR(stepper_base[i],REGISTER::outputMax,max_speed[i]);
                IOWR(stepper_base[i],REGISTER::enable,1);
                Zero(i);
        }

        // cleanser stepper
        IOWR(stepper_base[3],REGISTER::ms,STEP_16);
        IOWR(stepper_base[3],REGISTER::Kp,1);
        IOWR(stepper_base[3],REGISTER::outputMax,6000);
        IOWR(stepper_base[3],REGISTER::enable,1);

        spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(0));
        spinner->start();

        motor_state_thread = boost::shared_ptr<std::thread>( new std::thread(&CNC::MotorStatePublisher, this));
        motor_state_thread->detach();

        zero_srv = nh->advertiseService("/zero", &CNC::ZeroService, this);

        std_msgs::ColorRGBA msg;

        ros::Rate rate(5);
        for(int i=0; i<2; i++) {
                for(int i=0; i<16; i++) {
                        IOWR(neopixel,i+1,255);
                }
                IOWR(neopixel,0,1);
                rate.sleep();
                for(int i=0; i<16; i++) {
                        IOWR(neopixel,i+1,0);
                }
                IOWR(neopixel,0,1);
                rate.sleep();
        }
        ROS_INFO("cnc initialized for %d motors", number_of_motors);
};

void CleanserCallback(const std_msgs::Int32ConstPtr &msg){
        IOWR(stepper_base[3],REGISTER::setpoint,msg->data);
}

bool ZeroService(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res){
        ROS_INFO("zero service called");
        bool timeout = false;
        for(int axis=number_of_axis-1; axis>=0; axis--) {
                if(!Zero(axis)) {
                        timeout = true;
                }
        }
        return !timeout;
}

bool Zero(int motor){
        ros::Time t0 = ros::Time::now();
        bool timeout = false;
        // we move util endswitch is triggered or timeout
        while(IORD(stepper_base[motor],REGISTER::endswitch) && !timeout && ros::ok()) {
                int current_pos = IORD(stepper_base[motor],REGISTER::position);
                IOWR(stepper_base[motor],REGISTER::setpoint,current_pos+10000);
                timeout = (ros::Time::now()-t0).toSec()>30;
        }
        IOWR(stepper_base[motor],REGISTER::pos_offset,1);
        switch (motor) {
        case 0:
                IOWR(stepper_base[0],REGISTER::setpoint,int(((min_position[0]+1)*axis_sign[0]-axis_position_offset[0])/MM_PER_TICK));
                break;
        case 1:
                IOWR(stepper_base[1],REGISTER::setpoint,int(((max_position[1]-1)*axis_sign[1]-axis_position_offset[1])/MM_PER_TICK));
                break;
        case 2:
                IOWR(stepper_base[2],REGISTER::setpoint,int(((max_position[2]-1)*axis_sign[2]-axis_position_offset[2])/MM_PER_TICK));
                break;
        }
        if(timeout) {
                ROS_WARN("timeout on axis %d, check endswitch!!", motor);
                return false;
        }
        return true;
}

void MotorStatePublisher(){
        vector<double> pos_prev;
        ros::Time t0 = ros::Time::now(), t1 = ros::Time::now();
        motor_state.position.resize(number_of_motors);
        motor_state.velocity.resize(number_of_motors);
        motor_state.effort.resize(number_of_motors);
        vector<int> endswitch;
        endswitch.resize(number_of_motors);
        geometry_msgs::Vector3 msg;
        ros::Rate rate(100);
        while(ros::ok()) {
                motor_state.header.seq++;
                motor_state.header.stamp = ros::Time::now();
                for(int i=0; i<number_of_motors; i++) {
                        motor_state.position[i] = int(IORD(stepper_base[i],REGISTER::position))*MM_PER_TICK*axis_sign[i]+axis_position_offset[i]; // 500000 ticks for 243mm
                        motor_state.velocity[i] = int(IORD(stepper_base[i],REGISTER::ticks_per_millisecond))*MM_PER_TICK; // millimeter per millisecond
                        endswitch[i] = int(IORD(stepper_base[i],REGISTER::endswitch));
                }
                msg.x = endswitch[0];
                msg.y = endswitch[1];
                msg.z = endswitch[2];
                // ROS_INFO_THROTTLE(1,"%d %d %d", endswitch[0], endswitch[1], endswitch[2]);
                endswitch_pub.publish(msg);
                motor_state_pub.publish(motor_state);
                t0 = motor_state.header.stamp;
                rate.sleep();
        }
}

void PositionCommandCallback(const geometry_msgs::Vector3ConstPtr &msg){
        if(number_of_motors>=3) {
                if(msg->x<=max_position[0] && msg->x>=min_position[0]) // softlimits
                        IOWR(stepper_base[0],REGISTER::setpoint,int((msg->x*axis_sign[0]-axis_position_offset[0])/MM_PER_TICK));
                else{
                        if(msg->x>max_position[0]) {
                                IOWR(stepper_base[0],REGISTER::setpoint,int((max_position[0]*axis_sign[0]-axis_position_offset[0])/MM_PER_TICK));
                        }else if(msg->x<min_position[0]) {
                                IOWR(stepper_base[0],REGISTER::setpoint,int((min_position[0]*axis_sign[0]-axis_position_offset[0])/MM_PER_TICK));
                        }
                        ROS_WARN_THROTTLE(1,"position command=%f for x-axis exceeds softlimits (%f,%f)", msg->x, min_position[0], max_position[0]);
                }
                if(msg->y<=max_position[1] && msg->y>=min_position[1]) // softlimits
                        IOWR(stepper_base[1],REGISTER::setpoint,int((msg->y*axis_sign[1]-axis_position_offset[1])/MM_PER_TICK));
                else{
                        if(msg->y>max_position[1]) {
                                IOWR(stepper_base[1],REGISTER::setpoint,int((max_position[1]*axis_sign[0]-axis_position_offset[0])/MM_PER_TICK));
                        }else if(msg->y<min_position[1]) {
                                IOWR(stepper_base[1],REGISTER::setpoint,int((min_position[1]*axis_sign[0]-axis_position_offset[0])/MM_PER_TICK));
                        }
                        ROS_WARN_THROTTLE(1,"position command=%f for y-axis exceeds softlimits (%f,%f), ignoring...", msg->y, min_position[1], max_position[1]);
                }
                if(msg->z<=max_position[2] && msg->z>=min_position[2]) // softlimits
                        IOWR(stepper_base[2],REGISTER::setpoint,int((msg->z*axis_sign[2]-axis_position_offset[2])/MM_PER_TICK));
                else{
                        if(msg->z>max_position[2]) {
                                IOWR(stepper_base[2],REGISTER::setpoint,int((max_position[2]*axis_sign[0]-axis_position_offset[0])/MM_PER_TICK));
                        }else if(msg->z<min_position[2]) {
                                IOWR(stepper_base[2],REGISTER::setpoint,int((min_position[2]*axis_sign[0]-axis_position_offset[0])/MM_PER_TICK));
                        }
                        ROS_WARN_THROTTLE(1,"position command=%f for z-axis exceeds softlimits (%f,%f), ignoring...", msg->z, min_position[2], max_position[2]);
                }
        }
}

void NeopixelAllCallback(const std_msgs::ColorRGBAConstPtr &msg){
        uint8_t r = 255*msg->r;
        uint8_t g = 255*msg->g;
        uint8_t b = 255*msg->b;
        for(int i=0; i<16; i++) {
                IOWR(neopixel,i+1,(b<<16|r<<8|g));
        }
        IOWR(neopixel,0,1);
}

void NeopixelSingleCallback(const sensor_msgs::ChannelFloat32ConstPtr &msg){
        for(int i=0; i<msg->values.size(); i++) {
                IOWR(neopixel,i+1,msg->values[i]);
        }
        IOWR(neopixel,0,1);
}

ros::NodeHandlePtr nh;
boost::shared_ptr<ros::AsyncSpinner> spinner;
ros::Publisher motor_state_pub, pid_state_pub, endswitch_pub;
ros::Subscriber position_sub, neopixel_single_sub, neopixel_all_sub, cleanser_sub;
ros::ServiceServer zero_srv;
boost::shared_ptr<std::thread> motor_state_thread;
sensor_msgs::JointState motor_state;
vector<control_msgs::JointControllerState> pid_params;
vector<control_msgs::PidState> pid_state;
vector<int32_t*>  stepper_base;
int32_t *end_switches, *neopixel;
int number_of_motors = 0, control_mode = -1;
const int number_of_axis = 3;
vector<int> position_offset;
const vector<int> setpoint_delta_axis = {3,4,1},
                  ramp_up_limits = {6000,6000,6000},
                  ramp_up_thresholds = {30,30,30},
                  max_speed = {13000,13000,14000};
const vector<float> max_position = {160,398,0},
                    min_position = {0,0,-52},
                    axis_sign = {-1,1,1},
                    axis_position_offset = {0,398,0};

enum REGISTER {
        setpoint = 0x0,
        Kp = 0x1,
        Ki = 0x2,
        deadband = 0x3,
        integralMax = 0x4,
        outputMax = 0x5,
        error = 0x6,
        position = 0x7,
        pterm = 0x8,
        iterm = 0x9,
        term_sum = 0xA,
        result = 0xB,
        pos_offset = 0xC,
        endswitch = 0xD,
        ticks_per_millisecond = 0xE,
        enable = 0xF,
        ms = 0x10,
        result_freq = 0x11,
        ramp_up_limit = 0x12,
        ramp_up_threshold = 0x13
};
enum MODESELECT {
        STEP_8,
        STEP_2,
        STEP_4,
        STEP_16
};
};
