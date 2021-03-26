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
        joystick_sub = nh->subscribe("/joy",1,&CNC::JoyCallback,this);
        position_sub = nh->subscribe("/motor/command",1,&CNC::PositionCommandCallback,this);
        neopixel_single_sub = nh->subscribe("/neopixel/single",1,&CNC::NeopixelSingleCallback,this);
        neopixel_all_sub = nh->subscribe("/neopixel/all",1,&CNC::NeopixelAllCallback,this);

        number_of_motors = stepper_base.size();

        position_offset.resize(number_of_motors);

        for(int i=number_of_motors-1; i>=0; i--) {
                IOWR(stepper_base[i],REGISTER::enable,1);
                IOWR(stepper_base[i],REGISTER::Kp,1);
                IOWR(stepper_base[i],REGISTER::Ki,1);
                IOWR(stepper_base[i],REGISTER::deadband,20);
                IOWR(stepper_base[i],REGISTER::integralMax,10);
                IOWR(stepper_base[i],REGISTER::outputMax,max_speed[i]);
                // Zero(i);
        }

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

};

bool ZeroService(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res){
        ROS_INFO("zero service called");
        bool timeout = false;
        for(int motor=number_of_motors-1; motor>=0; motor--) {
                if(!Zero(motor)) {
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
        IOWR(stepper_base[motor],REGISTER::zero,1);
        IOWR(stepper_base[motor],REGISTER::setpoint,0);
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
        ros::Rate rate(100);
        while(ros::ok()) {
                motor_state.header.seq++;
                motor_state.header.stamp = ros::Time::now();
                for(int i=0; i<number_of_motors; i++) {
                        motor_state.position[i] = int(IORD(stepper_base[i],REGISTER::position))*MM_PER_TICK*axis_sign[i]+axis_position_offset[i]; // 500000 ticks for 243mm
                        motor_state.velocity[i] = int(IORD(stepper_base[i],REGISTER::position))*MM_PER_TICK; // millimeter per millisecond
                        // if((ros::Time::now()-t1).toSec()>3 && i==1){
                        //     ROS_INFO_THROTTLE(1,
                        //       "setpoint %d\n"
                        //       "Kp %d\n"
                        //       "Ki %d\n"
                        //       "deadband %d\n"
                        //       "integralMax %d\n"
                        //       "outputMax %d\n"
                        //       "error %d\n"
                        //       "position %d\n"
                        //       "pterm %d\n"
                        //       "iterm %d\n"
                        //       "result %d",
                        //       IORD(stepper_base[i],REGISTER::setpoint), IORD(stepper_base[i],REGISTER::Kp), IORD(stepper_base[i],REGISTER::Ki),
                        //       IORD(stepper_base[i],REGISTER::deadband), IORD(stepper_base[i],REGISTER::integralMax), IORD(stepper_base[i],REGISTER::outputMax),
                        //       IORD(stepper_base[i],REGISTER::error), IORD(stepper_base[i],REGISTER::position), IORD(stepper_base[i],REGISTER::pterm),
                        //       IORD(stepper_base[i],REGISTER::iterm), IORD(stepper_base[i],REGISTER::result)
                        //   );
                        //   t1 = ros::Time::now();
                        // }
                }
                motor_state_pub.publish(motor_state);
                t0 = motor_state.header.stamp;
                rate.sleep();
        }
}

void PositionCommandCallback(const geometry_msgs::Vector3ConstPtr &msg){
        if(number_of_motors>=3) {
                if(msg->x<=max_position[0] && msg->x>=min_position[0]) // softlimits
                        IOWR(stepper_base[0],REGISTER::setpoint,int((msg->x*axis_sign[0]-axis_position_offset[0])/MM_PER_TICK));
                else
                        ROS_WARN_THROTTLE(1,"position command=%f for x-axis exceeds softlimits (%f,%f), ignoring...", msg->x, min_position[0], max_position[0]);
                if(msg->y<=max_position[1] && msg->y>=min_position[1]) // softlimits
                        IOWR(stepper_base[1],REGISTER::setpoint,int((msg->y*axis_sign[1]-axis_position_offset[1])/MM_PER_TICK));
                else
                        ROS_WARN_THROTTLE(1,"position command=%f for y-axis exceeds softlimits (%f,%f), ignoring...", msg->y, min_position[1], max_position[1]);
                if(msg->z<=max_position[2] && msg->z>=min_position[2]) // softlimits
                        IOWR(stepper_base[2],REGISTER::setpoint,int((msg->z*axis_sign[2]-axis_position_offset[2])/MM_PER_TICK));
                else
                        ROS_WARN_THROTTLE(1,"position command=%f for z-axis exceeds softlimits (%f,%f), ignoring...", msg->z, min_position[2], max_position[2]);
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

void JoyCallback(const sensor_msgs::JoyConstPtr &msg){
        bool button_a = msg->buttons[0];
        bool button_b = msg->buttons[1];
        bool button_x = msg->buttons[2];
        bool button_y = msg->buttons[3];
        bool button_start = msg->buttons[7];
        bool button_back = msg->buttons[6];
        if(button_b) {
                control_mode = -1;
        }

        if(button_a) {
                control_mode = POSITION;
                for(int i=0; i<number_of_motors; i++)
                        position_offset[i] = IORD(stepper_base[i],REGISTER::position);
                ROS_WARN("POSITION control activated");
        }
        // if(button_x){
        //   control_mode = VELOCITY;
        //   pid_params[i].set_point = 0;
        //   ROS_WARN("VELOCITY control activated");
        // }
        // if(button_y){
        //   control_mode = FORCE;
        //   pid_params[i].set_point = 0;
        //   ROS_WARN("FORCE control activated");
        // }

        for(int i=0; i<number_of_motors; i++) {
                switch(control_mode) {
                case POSITION: {
                        IOWR(stepper_base[i],REGISTER::setpoint,int(-msg->axes[setpoint_delta_axis[i]]*max_position[i]/MM_PER_TICK));
                        break;
                }
                default: {
                        ROS_WARN("controller deactivated, choose A: POSITION");
                }
                }
        }
}

ros::NodeHandlePtr nh;
boost::shared_ptr<ros::AsyncSpinner> spinner;
ros::Publisher motor_state_pub, pid_state_pub;
ros::Subscriber joystick_sub, position_sub, neopixel_single_sub, neopixel_all_sub;
ros::ServiceServer zero_srv;
boost::shared_ptr<std::thread> motor_state_thread;
sensor_msgs::JointState motor_state;
vector<control_msgs::JointControllerState> pid_params;
vector<control_msgs::PidState> pid_state;
vector<int32_t*>  stepper_base;
int32_t *end_switches, *neopixel;
int number_of_motors = 0, control_mode = -1;
vector<int> position_offset;
const vector<int> setpoint_delta_axis = {3,4,1},
                  max_speed = {6000,6000,6000};
const vector<float> max_position = {160,398,0},
                    min_position = {0,0,-45},
                    axis_sign = {-1,1,1},
                    axis_position_offset = {0,398,0};

enum REGISTER {
        setpoint,
        Kp,
        Ki,
        deadband,
        integralMax,
        outputMax,
        error,
        position,
        pterm,
        iterm,
        result,
        zero,
        endswitch,
        ticks_per_millisecond,
        enable,
        ms
};
};
