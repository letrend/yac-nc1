
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
                IOWR(h2p_lw_stepper_addr[i],1,0);
                IOWR(h2p_lw_stepper_addr[i],2,0);
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

        CNC cnc(h2p_lw_stepper_addr,h2p_lw_end_switch_addr,h2p_lw_neopixel_addr);

        ros::Rate rate(1);
        while(ros::ok()) {
                rate.sleep();
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
