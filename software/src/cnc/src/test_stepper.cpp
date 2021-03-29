
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <stdlib.h>
#include "hwlib.h"
#include "socal/hps.h"
#include "hps_0.h"
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include "cnc.hpp"

using namespace std;

#define SYSTEM_ID 0xb16b00b5

#define ALT_LWFPGASLVS_OFST 0xFF200000
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

int32_t *h2p_lw_sysid_addr;
vector<int32_t *> h2p_lw_stepper_addr;
vector<int32_t *> h2p_lw_quad_decoder_addr;
int32_t *h2p_lw_end_switch_addr;
int32_t *h2p_lw_neopixel_addr;

void SigintHandler(int sig){
        cout << "shutting down" << endl;
        ros::shutdown();
        for(int i=0; i<h2p_lw_stepper_addr.size(); i++) {
                IOWR(h2p_lw_stepper_addr[i],CNC::REGISTER::enable,0);
        }
        for(int i=0; i<16; i++) {
                IOWR(h2p_lw_neopixel_addr,i+1,255<<8);
        }
        IOWR(h2p_lw_neopixel_addr,0,1);

        system("fortune | cowsay");
        for(int i=0; i<16; i++) {
                IOWR(h2p_lw_neopixel_addr,i+1,0);
        }
        IOWR(h2p_lw_neopixel_addr,0,1);
}

int main(int argc, char *argv[]) {
        void *virtual_base;
        int fd;

//     map the address space for all registers into user space so we can interact with them.
//     we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span
        if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
                printf( "ERROR: could not open \"/dev/mem\"...\n" );
                return( 1 );
        }

        virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

        if( virtual_base == MAP_FAILED ) {
                printf( "ERROR: mmap() failed...\n" );
                close( fd );
                return( 1 );
        }

        h2p_lw_sysid_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SYSID_QSYS_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
        if(*h2p_lw_sysid_addr!=SYSTEM_ID) { // if the system id does not match, we abort
                printf("system id %x does not match this version %x, make sure you loaded the correct fpga image\n",*h2p_lw_sysid_addr, SYSTEM_ID);
                // clean up our memory mapping and exit
                if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
                        printf( "ERROR: munmap() failed...\n" );
                        close( fd );
                        return( 1 );
                }
                close( fd );
                return -1;
        }

    #ifdef STEPPER_POSITION_CONTOLLER_0_BASE
        h2p_lw_stepper_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + STEPPER_POSITION_CONTOLLER_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    #endif
    #ifdef STEPPER_POSITION_CONTOLLER_1_BASE
        h2p_lw_stepper_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + STEPPER_POSITION_CONTOLLER_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    #endif
    #ifdef STEPPER_POSITION_CONTOLLER_2_BASE
        h2p_lw_stepper_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + STEPPER_POSITION_CONTOLLER_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    #endif

    #ifdef ENDSWITCHES_BASE
        h2p_lw_end_switch_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ENDSWITCHES_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    #endif

    #ifdef NEOPIXEL_0_BASE
        h2p_lw_neopixel_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + NEOPIXEL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    #endif

        signal(SIGINT, SigintHandler);

        if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "cnc", ros::init_options::NoSigintHandler);
                ros::start();
        }

        enum MODESELECT {
                STEP_8,
                STEP_2,
                STEP_4,
                STEP_16
        };


        IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::Kp,1);
        IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::Ki,0);
        IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::deadband,0);
        IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::outputMax,5000);
        IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::pos_offset,1);
        IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::enable,1);

        // IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::setpoint,-100000000);
        // IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::ms,STEP_16);
        // vector<int> vels;
        // int samples = 100;
        // ros::Rate rate(1000);
        // for(int i=0; i<15000; i+=1) {
        //         ROS_INFO_THROTTLE(1,"freq: %d",i);
        //         IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::outputMax,i);
        //         int vel= IORD(h2p_lw_stepper_addr[0],CNC::REGISTER::ticks_per_millisecond);
        //         vels.push_back(abs(vel));
        //         if(!ros::ok())
        //                 break;
        //         rate.sleep();
        // }
        //
        // for(auto v:vels) {
        //         ROS_INFO("%d",v);
        // }
        //
        // auto it = max_element(std::begin(vels), std::end(vels));
        // ROS_INFO("max %d with vel %d",(it-vels.begin())*10,*it);

        int current_setpoint = 0;
        vector<int> setpoints = {0,-100000,0,100000};

        ros::Rate rate(1);
        while(ros::ok()) {
                // int vel = IORD(h2p_lw_stepper_addr[0],CNC::REGISTER::ticks_per_millisecond);
                // int vel_abs = abs(vel);
                // if(vel_abs<5) {
                //         IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::ms,STEP_16);
                // }else if(vel_abs<10) {
                //         IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::ms,STEP_8);
                // }else if(vel_abs<30) {
                //         IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::ms,STEP_4);
                // }else{
                //         IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::ms,STEP_2);
                // }

                ROS_INFO_THROTTLE(0.1,
                                  "sp %d\n"
                                  "pos %d\n"
                                  "vel: %d\n"
                                  "error: %d\n"
                                  "pterm: %d\n"
                                  "iterm: %d\n"
                                  "result: %d\n"
                                  "ms: %d\n"
                                  "result_freq: %d\n"
                                  ,
                                  IORD(h2p_lw_stepper_addr[0],CNC::REGISTER::setpoint),
                                  IORD(h2p_lw_stepper_addr[0],CNC::REGISTER::position),
                                  IORD(h2p_lw_stepper_addr[0],CNC::REGISTER::ticks_per_millisecond),
                                  IORD(h2p_lw_stepper_addr[0],CNC::REGISTER::error),
                                  IORD(h2p_lw_stepper_addr[0],CNC::REGISTER::pterm),
                                  IORD(h2p_lw_stepper_addr[0],CNC::REGISTER::iterm),
                                  IORD(h2p_lw_stepper_addr[0],CNC::REGISTER::result),
                                  IORD(h2p_lw_stepper_addr[0],CNC::REGISTER::ms),
                                  IORD(h2p_lw_stepper_addr[0],CNC::REGISTER::result_freq)
                                  );

                int error = IORD(h2p_lw_stepper_addr[0],CNC::REGISTER::error);
                int error_abs = abs(error);
                if(error_abs<30) {
                        IOWR(h2p_lw_stepper_addr[0],CNC::REGISTER::setpoint,setpoints[current_setpoint]);
                        current_setpoint++;
                        if(current_setpoint>=setpoints.size())
                                current_setpoint = 0;
                }
        }

        // clean up our memory mapping and exit
        if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
                printf( "ERROR: munmap() failed...\n" );
                close( fd );
                return( 1 );
        }

        close( fd );

        return( 0 );
}
